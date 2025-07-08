from pathlib import Path
import json
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.utils.data import DataLoader, random_split
import cv2
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt
from tqdm import tqdm
import os

# Use ROS2 logger
import rclpy
from rclpy.logging import get_logger

# Initialize ROS2 (if not already initialized)
try:
    rclpy.init()
except RuntimeError:
    pass  # Already initialized

logger = get_logger('vision_control_trainer')

from .collector import VisionControlDataset


class VisionControlNet(nn.Module):
    """Vision Control Network - CNN for image-to-control mapping"""
    
    def __init__(self, input_shape=(3, 224, 224), hidden_dim=512, output_dim=2):
        super(VisionControlNet, self).__init__()
        
        # CNN feature extractor
        self.conv1 = nn.Conv2d(3, 32, kernel_size=8, stride=4, padding=2)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1)
        self.conv4 = nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1)
        
        # Calculate the size of flattened features
        self.feature_size = self._get_conv_output_size(input_shape)
        
        # Fully connected layers
        self.fc1 = nn.Linear(self.feature_size, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim // 2)
        self.fc3 = nn.Linear(hidden_dim // 2, output_dim)
        
        # Dropout for regularization
        self.dropout = nn.Dropout(0.3)
        
    def _get_conv_output_size(self, shape):
        """Calculate the output size of convolutional layers"""
        with torch.no_grad():
            input_tensor = torch.zeros(1, *shape)
            output = self._forward_conv(input_tensor)
            return output.numel()
    
    def _forward_conv(self, x):
        """Forward pass through convolutional layers"""
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.conv4(x))
        return x
    
    def forward(self, x):
        # Convolutional layers
        x = self._forward_conv(x)
        
        # Flatten
        x = x.view(x.size(0), -1)
        
        # Fully connected layers
        x = F.relu(self.fc1(x))
        x = self.dropout(x)
        x = F.relu(self.fc2(x))
        x = self.dropout(x)
        x = self.fc3(x)
        
        return x


class VisionControlTrainer:
    """Trainer for Vision Control models"""
    
    def __init__(self, 
                 dataset_dir: str,
                 model_name: str = "vision_control_model",
                 out_dir: str = None,
                 device: str = "auto"):
        
        self.dataset_dir = Path(dataset_dir)
        self.model_name = model_name
        
        # Set output directory
        if out_dir is None:
            out_dir = os.path.expanduser("~/.driveguard")
        self.out_dir = Path(out_dir)
        
        # Create models directory structure
        self.models_dir = self.out_dir / "models"
        self.models_dir.mkdir(parents=True, exist_ok=True)
        
        # Create model-specific directories
        self.model_dir = self.models_dir / self.model_name
        self.model_dir.mkdir(parents=True, exist_ok=True)
        
        self.metrics_dir = self.model_dir / "metrics"
        self.metrics_dir.mkdir(exist_ok=True)
        
        self.checkpoints_dir = self.model_dir / "checkpoints"
        self.checkpoints_dir.mkdir(exist_ok=True)
        
        # Device selection
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)
        
        logger.info(f"🔧 Using device: {self.device}")
        
        # Load dataset
        self.dataset = VisionControlDataset(str(self.dataset_dir))
        self.metadata = self.dataset.get_metadata()
        
        logger.info(f"📊 Loaded dataset: {self.metadata['dataset_name']}")
        logger.info(f"📈 Total samples: {len(self.dataset)}")
        
        # Training history (only for current session plotting)
        self.train_history = {
            'train_loss': [],
            'val_loss': [],
            'train_linear_loss': [],
            'train_angular_loss': [],
            'val_linear_loss': [],
            'val_angular_loss': [],
            'epochs': [],
            'learning_rates': []
        }
        
        # Load existing training info if available
        self.epoch_offset = 0
        self._load_existing_training_info()
        
    def _load_existing_training_info(self):
        """Load existing training info if model exists"""
        metadata_path = self.model_dir / "metadata.json"
        if metadata_path.exists():
            try:
                with open(metadata_path, 'r') as f:
                    existing_metadata = json.load(f)
                
                # Set epoch offset to continue from last training
                self.epoch_offset = existing_metadata.get('total_epochs', 0)
                logger.info(f"📈 Continuing from epoch {self.epoch_offset}")
                    
            except Exception as e:
                logger.warn(f"⚠️  Could not load existing training info: {e}")
                self.epoch_offset = 0
    
    def preprocess_image(self, image):
        """Preprocess image for training"""
        # Resize image
        image = cv2.resize(image, (224, 224))
        
        # Convert BGR to RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0, 1]
        image = image.astype(np.float32) / 255.0
        
        # Convert to tensor and change to CHW format
        image = torch.from_numpy(image).permute(2, 0, 1)
        
        return image
    
    def create_data_loaders(self, batch_size=32, val_split=0.2, num_workers=4):
        """Create training and validation data loaders"""
        
        # Custom collate function
        def collate_fn(batch):
            images = []
            controls = []
            
            for item in batch:
                image = self.preprocess_image(item['image'])
                control = item['control_command']
                
                images.append(image)
                controls.append([control['linear_x'], control['angular_z']])
            
            images = torch.stack(images)
            controls = torch.tensor(controls, dtype=torch.float32)
            
            return images, controls
        
        # Split dataset
        dataset_size = len(self.dataset)
        val_size = int(dataset_size * val_split)
        train_size = dataset_size - val_size
        
        train_dataset, val_dataset = random_split(
            self.dataset, 
            [train_size, val_size],
            generator=torch.Generator().manual_seed(42)
        )
        
        # Create data loaders
        train_loader = DataLoader(
            train_dataset, 
            batch_size=batch_size, 
            shuffle=True, 
            num_workers=num_workers,
            collate_fn=collate_fn
        )
        
        val_loader = DataLoader(
            val_dataset, 
            batch_size=batch_size, 
            shuffle=False, 
            num_workers=num_workers,
            collate_fn=collate_fn
        )
        
        logger.info(f"📊 Train samples: {len(train_dataset)}")
        logger.info(f"📊 Validation samples: {len(val_dataset)}")
        
        return train_loader, val_loader
    
    def train(self, 
              epochs=100,
              batch_size=32,
              learning_rate=0.001,
              val_split=0.2,
              save_interval=10,
              patience=50):
        """Train the vision control model"""
        
        logger.info(f"🚀 Starting training for {epochs} epochs...")
        if self.epoch_offset > 0:
            logger.info(f"📈 Continuing from epoch {self.epoch_offset}")
        
        # Create data loaders
        train_loader, val_loader = self.create_data_loaders(
            batch_size=batch_size, 
            val_split=val_split
        )
        
        # Create model
        model = VisionControlNet().to(self.device)
        
        # Load existing model if available
        best_model_path = self.model_dir / "best_model.pth"
        if best_model_path.exists():
            try:
                checkpoint = torch.load(best_model_path, map_location=self.device)
                model.load_state_dict(checkpoint['model_state_dict'])
                logger.info(f"📦 Loaded existing model from {best_model_path}")
            except Exception as e:
                logger.warn(f"⚠️  Could not load existing model: {e}")
        
        # Loss functions
        criterion = nn.MSELoss()
        linear_criterion = nn.MSELoss()
        angular_criterion = nn.MSELoss()
        
        # Optimizer and scheduler
        optimizer = optim.Adam(model.parameters(), lr=learning_rate)
        scheduler = optim.lr_scheduler.StepLR(optimizer, step_size=30, gamma=0.1)
        
        # Early stopping
        best_val_loss = float('inf')
        if best_model_path.exists():
            try:
                checkpoint = torch.load(best_model_path, map_location=self.device)
                best_val_loss = checkpoint.get('val_loss', float('inf'))
                logger.info(f"🏆 Previous best validation loss: {best_val_loss:.4f}")
            except Exception as e:
                logger.warn(f"⚠️  Could not load previous best loss: {e}")
        
        patience_counter = 0
        
        # Training loop
        for epoch in range(epochs):
            # Actual epoch number including offset
            actual_epoch = epoch + self.epoch_offset + 1
            
            # Training phase
            model.train()
            train_loss = 0.0
            train_linear_loss = 0.0
            train_angular_loss = 0.0
            train_batches = 0
            
            pbar = tqdm(train_loader, desc=f"Epoch {actual_epoch}/{epochs + self.epoch_offset}")
            for batch_idx, (images, controls) in enumerate(pbar):
                images, controls = images.to(self.device), controls.to(self.device)
                
                optimizer.zero_grad()
                outputs = model(images)
                
                # Calculate losses
                total_loss = criterion(outputs, controls)
                linear_loss = linear_criterion(outputs[:, 0], controls[:, 0])
                angular_loss = angular_criterion(outputs[:, 1], controls[:, 1])
                
                total_loss.backward()
                optimizer.step()
                
                train_loss += total_loss.item()
                train_linear_loss += linear_loss.item()
                train_angular_loss += angular_loss.item()
                train_batches += 1
                
                pbar.set_postfix({
                    'loss': f"{total_loss.item():.4f}",
                    'linear': f"{linear_loss.item():.4f}",
                    'angular': f"{angular_loss.item():.4f}"
                })
            
            # Validation phase
            model.eval()
            val_loss = 0.0
            val_linear_loss = 0.0
            val_angular_loss = 0.0
            val_batches = 0
            
            with torch.no_grad():
                for images, controls in val_loader:
                    images, controls = images.to(self.device), controls.to(self.device)
                    outputs = model(images)
                    
                    # Calculate losses
                    total_loss = criterion(outputs, controls)
                    linear_loss = linear_criterion(outputs[:, 0], controls[:, 0])
                    angular_loss = angular_criterion(outputs[:, 1], controls[:, 1])
                    
                    val_loss += total_loss.item()
                    val_linear_loss += linear_loss.item()
                    val_angular_loss += angular_loss.item()
                    val_batches += 1
            
            # Calculate average losses
            avg_train_loss = train_loss / train_batches
            avg_val_loss = val_loss / val_batches
            avg_train_linear_loss = train_linear_loss / train_batches
            avg_train_angular_loss = train_angular_loss / train_batches
            avg_val_linear_loss = val_linear_loss / val_batches
            avg_val_angular_loss = val_angular_loss / val_batches
            
            # Update history (only for current session)
            self.train_history['train_loss'].append(avg_train_loss)
            self.train_history['val_loss'].append(avg_val_loss)
            self.train_history['train_linear_loss'].append(avg_train_linear_loss)
            self.train_history['train_angular_loss'].append(avg_train_angular_loss)
            self.train_history['val_linear_loss'].append(avg_val_linear_loss)
            self.train_history['val_angular_loss'].append(avg_val_angular_loss)
            self.train_history['epochs'].append(actual_epoch)
            self.train_history['learning_rates'].append(scheduler.get_last_lr()[0])
            
            # Step scheduler
            scheduler.step()
            
            logger.info(f"📈 Epoch {actual_epoch}: Train Loss: {avg_train_loss:.4f}, Val Loss: {avg_val_loss:.4f}")
            
            # Save checkpoint
            if actual_epoch % save_interval == 0:
                self.save_checkpoint(model, optimizer, actual_epoch)
            
            # Early stopping and best model saving
            if avg_val_loss < best_val_loss:
                best_val_loss = avg_val_loss
                patience_counter = 0
                self.save_best_model(model, actual_epoch, avg_val_loss)
            else:
                patience_counter += 1
                if patience_counter >= patience:
                    logger.info(f"🛑 Early stopping at epoch {actual_epoch}")
                    break
        
        # Save final training metadata
        final_epoch = actual_epoch
        self.save_training_metadata(final_epoch, best_val_loss)
        
        # Generate training plots for current session
        self.plot_training_metrics()
        
        logger.info("🎉 Training completed!")
    
    def save_checkpoint(self, model, optimizer, epoch):
        """Save model checkpoint"""
        checkpoint_path = self.checkpoints_dir / f"checkpoint_epoch_{epoch}.pth"
        
        torch.save({
            'epoch': epoch,
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'model_name': self.model_name,
            'dataset_metadata': self.metadata
        }, checkpoint_path)
        
        logger.info(f"💾 Checkpoint saved: {checkpoint_path}")
    
    def save_best_model(self, model, epoch, val_loss):
        """Save best model"""
        best_model_path = self.model_dir / "best_model.pth"
        
        torch.save({
            'epoch': epoch,
            'model_state_dict': model.state_dict(),
            'val_loss': val_loss,
            'model_name': self.model_name,
            'dataset_metadata': self.metadata,
            'timestamp': datetime.now().isoformat()
        }, best_model_path)
        
        logger.info(f"🏆 Best model saved: {best_model_path} (Val Loss: {val_loss:.4f})")
    
    def save_training_metadata(self, final_epoch, best_val_loss):
        """Save training metadata"""
        metadata = {
            'model_name': self.model_name,
            'dataset_name': self.metadata['dataset_name'],
            'last_training_date': datetime.now().isoformat(),
            'total_epochs': final_epoch,
            'best_val_loss': best_val_loss,
            'total_parameters': sum(p.numel() for p in VisionControlNet().parameters()),
            'device': str(self.device),
            'dataset_size': len(self.dataset),
            'dataset_metadata': self.metadata
        }
        
        metadata_path = self.model_dir / "metadata.json"
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        logger.info(f"📄 Training metadata saved: {metadata_path}")
    
    def plot_training_metrics(self):
        """Generate and save training metrics plots with timestamp (current session only)"""
        # Create timestamp for filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Plot 1: Loss curves
        plt.figure(figsize=(15, 10))
        
        # Total loss
        plt.subplot(2, 2, 1)
        plt.plot(self.train_history['epochs'], self.train_history['train_loss'], 
                label='Training Loss', marker='o', linewidth=2)
        plt.plot(self.train_history['epochs'], self.train_history['val_loss'], 
                label='Validation Loss', marker='s', linewidth=2)
        plt.xlabel('Epoch')
        plt.ylabel('Total Loss')
        plt.title('Training and Validation Loss')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Linear velocity loss
        plt.subplot(2, 2, 2)
        plt.plot(self.train_history['epochs'], self.train_history['train_linear_loss'], 
                label='Training Linear Loss', marker='o', linewidth=2)
        plt.plot(self.train_history['epochs'], self.train_history['val_linear_loss'], 
                label='Validation Linear Loss', marker='s', linewidth=2)
        plt.xlabel('Epoch')
        plt.ylabel('Linear Velocity Loss')
        plt.title('Linear Velocity Loss')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Angular velocity loss
        plt.subplot(2, 2, 3)
        plt.plot(self.train_history['epochs'], self.train_history['train_angular_loss'], 
                label='Training Angular Loss', marker='o', linewidth=2)
        plt.plot(self.train_history['epochs'], self.train_history['val_angular_loss'], 
                label='Validation Angular Loss', marker='s', linewidth=2)
        plt.xlabel('Epoch')
        plt.ylabel('Angular Velocity Loss')
        plt.title('Angular Velocity Loss')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Learning rate
        plt.subplot(2, 2, 4)
        plt.plot(self.train_history['epochs'], self.train_history['learning_rates'], 
                marker='o', linewidth=2, color='red')
        plt.xlabel('Epoch')
        plt.ylabel('Learning Rate')
        plt.title('Learning Rate Schedule')
        plt.grid(True, alpha=0.3)
        plt.yscale('log')
        
        plt.tight_layout()
        plot_path = self.metrics_dir / f"training_metrics_{timestamp}.png"
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info(f"📊 Training metrics plot saved: {plot_path}")
    
    def evaluate(self, model_path: str = None):
        """Evaluate a trained model"""
        if model_path is None:
            model_path = str(self.model_dir / "best_model.pth")
        
        # Load model
        checkpoint = torch.load(model_path, map_location=self.device)
        model = VisionControlNet().to(self.device)
        model.load_state_dict(checkpoint['model_state_dict'])
        model.eval()
        
        # Create test data loader
        test_loader = DataLoader(
            self.dataset, 
            batch_size=32, 
            shuffle=False,
            collate_fn=lambda batch: (
                torch.stack([self.preprocess_image(item['image']) for item in batch]),
                torch.tensor([[item['control_command']['linear_x'], 
                              item['control_command']['angular_z']] for item in batch], 
                            dtype=torch.float32)
            )
        )
        
        # Evaluate
        total_loss = 0.0
        linear_loss = 0.0
        angular_loss = 0.0
        total_samples = 0
        
        criterion = nn.MSELoss()
        
        with torch.no_grad():
            for images, controls in test_loader:
                images, controls = images.to(self.device), controls.to(self.device)
                outputs = model(images)
                
                batch_loss = criterion(outputs, controls)
                batch_linear_loss = criterion(outputs[:, 0], controls[:, 0])
                batch_angular_loss = criterion(outputs[:, 1], controls[:, 1])
                
                total_loss += batch_loss.item() * images.size(0)
                linear_loss += batch_linear_loss.item() * images.size(0)
                angular_loss += batch_angular_loss.item() * images.size(0)
                total_samples += images.size(0)
        
        avg_loss = total_loss / total_samples
        avg_linear_loss = linear_loss / total_samples
        avg_angular_loss = angular_loss / total_samples
        
        logger.info(f"📊 Evaluation Results:")
        logger.info(f"  Total Loss: {avg_loss:.4f}")
        logger.info(f"  Linear Loss: {avg_linear_loss:.4f}")
        logger.info(f"  Angular Loss: {avg_angular_loss:.4f}")
        logger.info(f"  Total Samples: {total_samples}")
        
        return {
            'total_loss': avg_loss,
            'linear_loss': avg_linear_loss,
            'angular_loss': avg_angular_loss,
            'total_samples': total_samples
        }