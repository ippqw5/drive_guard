import os
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, random_split
import numpy as np
import time
import matplotlib.pyplot as plt
from datetime import datetime
import yaml
import argparse

from .models.resnet import DriveGuardResNet18
from .collector import DriveGuardDataset
from .config import *


class ModelTrainer:
    """Model trainer for DriveGuard neural network"""
    
    def __init__(self, dataset_name=None):
        # Use configuration from config.py
        self.training_config = TRAINING

        # Fixed directory structure
        self.datasets_dir = DATASETS_DIR
        self.models_dir = MODELS_DIR

        # Determine dataset name
        if dataset_name:
            self.dataset_name = dataset_name
            self.dataset_path = self._get_specific_dataset(dataset_name)
        else:
            self.dataset_name, self.dataset_path = self._find_latest_dataset()
        
        # Setup model save directory based on dataset name
        self.model_save_dir = os.path.join(self.models_dir, self.dataset_name)
        os.makedirs(self.model_save_dir, exist_ok=True)
        
        # Training parameters from config
        self.batch_size = self.training_config['batch_size']
        self.learning_rate = self.training_config['learning_rate']
        self.num_epochs = self.training_config['num_epochs']
        self.val_split = self.training_config['val_split']
        self.image_size = tuple(self.training_config['image_size'])
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        print(f"ModelTrainer initialized:")
        print(f"  Device: {self.device}")
        print(f"  Using dataset: {self.dataset_name}")
        print(f"  Dataset file: {self.dataset_path}")
        print(f"  Model save directory: {self.model_save_dir}")
        print(f"  Batch size: {self.batch_size}")
        print(f"  Learning rate: {self.learning_rate}")
        print(f"  Epochs: {self.num_epochs}")
        print(f"  Image size: {self.image_size}")
    
    def _get_specific_dataset(self, dataset_name):
        """Get path to specific dataset"""
        dataset_dir = os.path.join(self.datasets_dir, dataset_name)
        if not os.path.exists(dataset_dir):
            raise FileNotFoundError(f'Dataset directory not found: {dataset_dir}')
        
        dataset_file = os.path.join(dataset_dir, 'processed_dataset.pt')
        if not os.path.exists(dataset_file):
            raise FileNotFoundError(f'Processed dataset not found: {dataset_file}')
        
        return dataset_file
    
    def _find_latest_dataset(self):
        """Find the latest processed dataset in the datasets directory"""
        if not os.path.exists(self.datasets_dir):
            raise FileNotFoundError(f'Datasets directory does not exist: {self.datasets_dir}')
            
        # Look for dataset directories with processed datasets
        dataset_candidates = []
        for item in os.listdir(self.datasets_dir):
            item_path = os.path.join(self.datasets_dir, item)
            if os.path.isdir(item_path):
                # Check if processed_dataset.pt exists
                dataset_file = os.path.join(item_path, 'processed_dataset.pt')
                if os.path.exists(dataset_file):
                    dataset_candidates.append((item, dataset_file))
        
        if not dataset_candidates:
            raise FileNotFoundError(f'No processed datasets found in {self.datasets_dir}')
            
        # Return the latest dataset based on name (should contain timestamp)
        latest_name, latest_path = max(dataset_candidates, key=lambda x: x[0])
        return latest_name, latest_path
    
    def train(self, dataset_path=None):
        """Start training process"""
        if dataset_path:
            self.dataset_path = dataset_path
            
        if not self.dataset_path:
            print('Error: No dataset found to train on')
            return None
            
        print(f'Loading dataset from {self.dataset_path}')
        
        try:
            # Load dataset with weights_only=False to handle ROS messages
            checkpoint = torch.load(self.dataset_path, map_location=self.device, weights_only=False)
            images = checkpoint['images']
            commands = checkpoint['commands']
            
            print(f'Dataset loaded: {len(images)} samples')
            
            # Use the image_size from the saved dataset if available, otherwise use config
            dataset_image_size = checkpoint.get('image_size', self.image_size)
            dataset = DriveGuardDataset(images, commands, dataset_image_size)
            
            # Split dataset
            val_size = int(self.val_split * len(dataset))
            train_size = len(dataset) - val_size
            train_dataset, val_dataset = random_split(dataset, [train_size, val_size])
            
            print(f'Dataset split: {train_size} training, {val_size} validation samples')
            
            train_loader = DataLoader(train_dataset, batch_size=self.batch_size, shuffle=True, num_workers=4)
            val_loader = DataLoader(val_dataset, batch_size=self.batch_size, shuffle=False, num_workers=4)
            
            # Initialize model
            model = DriveGuardResNet18(num_outputs=2)  # Linear velocity and angular velocity
            model = model.to(self.device)
            
            print(f'Model initialized: {sum(p.numel() for p in model.parameters())} parameters')
            
            # Loss function and optimizer
            criterion = nn.MSELoss()
            optimizer = optim.Adam(model.parameters(), lr=self.learning_rate)
            
            # Training loop
            print(f'Starting training for {self.num_epochs} epochs')
            
            train_losses = []
            val_losses = []
            
            for epoch in range(self.num_epochs):
                start_time = time.time()
                
                # Training
                model.train()
                running_loss = 0.0
                for i, (inputs, targets) in enumerate(train_loader):
                    inputs = inputs.to(self.device)
                    targets = targets.to(self.device)
                    
                    # Forward pass
                    optimizer.zero_grad()
                    outputs = model(inputs)
                    loss = criterion(outputs, targets)
                    
                    # Backward and optimize
                    loss.backward()
                    optimizer.step()
                    
                    running_loss += loss.item()
                    
                    if (i+1) % 10 == 0:
                        print(f'Epoch [{epoch+1}/{self.num_epochs}], Step [{i+1}/{len(train_loader)}], Loss: {loss.item():.6f}')
                
                train_loss = running_loss / len(train_loader)
                train_losses.append(train_loss)
                
                # Validation
                model.eval()
                val_loss = 0.0
                with torch.no_grad():
                    for inputs, targets in val_loader:
                        inputs = inputs.to(self.device)
                        targets = targets.to(self.device)
                        outputs = model(inputs)
                        loss = criterion(outputs, targets)
                        val_loss += loss.item()
                
                val_loss = val_loss / len(val_loader)
                val_losses.append(val_loss)
                
                epoch_time = time.time() - start_time
                print(f'Epoch {epoch+1}/{self.num_epochs} completed in {epoch_time:.2f}s | '
                      f'Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}')
            
            # Save model with timestamp in filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            model_filename = f'model_{timestamp}.pt'
            model_save_path = os.path.join(self.model_save_dir, model_filename)
            
            # Save complete model checkpoint
            torch.save({
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'train_losses': train_losses,
                'val_losses': val_losses,
                'epochs': self.num_epochs,
                'final_train_loss': train_losses[-1],
                'final_val_loss': val_losses[-1],
                'dataset_name': self.dataset_name,
                'dataset_path': self.dataset_path,
                'model_config': {
                    'batch_size': self.batch_size,
                    'learning_rate': self.learning_rate,
                    'image_size': self.image_size,
                    'dataset_image_size': dataset_image_size
                },
                'training_config': self.training_config,
                'timestamp': timestamp
            }, model_save_path)
            
            print(f'Model saved to {model_save_path}')
            
            # Plot training curve
            curve_path = model_save_path.replace('.pt', '_training_curve.png')
            self._plot_training_curves(train_losses, val_losses, curve_path)
            
            # Save training summary
            self._save_training_summary(model_save_path, train_losses, val_losses)
            
            # Create a symlink to the latest model for easy access
            latest_model_link = os.path.join(self.model_save_dir, 'latest_model.pt')
            if os.path.exists(latest_model_link):
                os.remove(latest_model_link)
            os.symlink(os.path.basename(model_save_path), latest_model_link)
            
            return model_save_path
            
        except Exception as e:
            print(f'Error during training: {str(e)}')
            raise
    
    def _plot_training_curves(self, train_losses, val_losses, save_path):
        """Plot and save training curves"""
        plt.figure(figsize=(12, 8))
        
        # Plot losses
        plt.subplot(2, 2, 1)
        plt.plot(train_losses, label='Training Loss', color='blue')
        plt.plot(val_losses, label='Validation Loss', color='red')
        plt.xlabel('Epochs')
        plt.ylabel('Loss')
        plt.title('Training and Validation Loss')
        plt.legend()
        plt.grid(True)
        
        # Plot loss difference
        plt.subplot(2, 2, 2)
        loss_diff = [val - train for train, val in zip(train_losses, val_losses)]
        plt.plot(loss_diff, label='Val - Train Loss', color='orange')
        plt.xlabel('Epochs')
        plt.ylabel('Loss Difference')
        plt.title('Overfitting Monitor')
        plt.legend()
        plt.grid(True)
        
        # Plot learning rate (if using scheduler)
        plt.subplot(2, 2, 3)
        plt.plot(train_losses, color='blue')
        plt.xlabel('Epochs')
        plt.ylabel('Training Loss')
        plt.title('Training Loss Trend')
        plt.yscale('log')
        plt.grid(True)
        
        # Final statistics
        plt.subplot(2, 2, 4)
        plt.text(0.1, 0.8, f'Final Train Loss: {train_losses[-1]:.6f}', fontsize=12)
        plt.text(0.1, 0.6, f'Final Val Loss: {val_losses[-1]:.6f}', fontsize=12)
        plt.text(0.1, 0.4, f'Best Val Loss: {min(val_losses):.6f}', fontsize=12)
        plt.text(0.1, 0.2, f'Total Epochs: {len(train_losses)}', fontsize=12)
        plt.axis('off')
        plt.title('Training Summary')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close()
        
        print(f'Training curve saved to {save_path}')
    
    def _save_training_summary(self, model_path, train_losses, val_losses):
        """Save training summary as text file"""
        summary_path = model_path.replace('.pt', '_summary.txt')
        
        with open(summary_path, 'w') as f:
            f.write("DriveGuard Model Training Summary\n")
            f.write("=" * 40 + "\n\n")
            f.write(f"Model Path: {model_path}\n")
            f.write(f"Device: {self.device}\n")
            f.write(f"Dataset Name: {self.dataset_name}\n")
            f.write(f"Dataset Path: {self.dataset_path}\n")
            f.write(f"Datasets Directory: {self.datasets_dir}\n")
            f.write(f"Batch Size: {self.batch_size}\n")
            f.write(f"Learning Rate: {self.learning_rate}\n")
            f.write(f"Epochs: {self.num_epochs}\n")
            f.write(f"Validation Split: {self.val_split}\n")
            f.write(f"Image Size: {self.image_size}\n\n")
            
            f.write("Training Results:\n")
            f.write("-" * 20 + "\n")
            f.write(f"Final Training Loss: {train_losses[-1]:.6f}\n")
            f.write(f"Final Validation Loss: {val_losses[-1]:.6f}\n")
            f.write(f"Best Validation Loss: {min(val_losses):.6f}\n")
            f.write(f"Best Epoch: {val_losses.index(min(val_losses)) + 1}\n")
            
        print(f'Training summary saved to {summary_path}')

    @classmethod
    def list_available_datasets(cls):
        """List all available processed datasets"""
        datasets_dir = DATASETS_DIR
        
        if not os.path.exists(datasets_dir):
            print(f"Datasets directory not found: {datasets_dir}")
            return []
        
        datasets = []
        for item in os.listdir(datasets_dir):
            item_path = os.path.join(datasets_dir, item)
            if os.path.isdir(item_path):
                dataset_file = os.path.join(item_path, 'processed_dataset.pt')
                metadata_file = os.path.join(item_path, 'dataset_metadata.yaml')
                
                if os.path.exists(dataset_file):
                    # Load metadata if available
                    metadata = {}
                    if os.path.exists(metadata_file):
                        try:
                            with open(metadata_file, 'r') as f:
                                metadata = yaml.safe_load(f)
                        except Exception as e:
                            metadata = {}
                    
                    # Get values with fallbacks
                    num_samples = metadata.get('num_samples', 'Unknown')
                    processed_at = metadata.get('processed_at', 'Unknown')
                    
                    # If metadata is missing, try to get info from dataset file
                    if num_samples == 'Unknown':
                        try:
                            checkpoint = torch.load(dataset_file, map_location='cpu', weights_only=False)
                            if 'images' in checkpoint:
                                num_samples = len(checkpoint['images'])
                        except Exception as e:
                            num_samples = 'Error'
                    
                    datasets.append({
                        'name': item,
                        'path': item_path,
                        'dataset_file': dataset_file,
                        'num_samples': num_samples,
                        'processed_at': processed_at
                    })
        
        return datasets

    @classmethod
    def list_available_models(cls, dataset_name=None):
        """List all available trained models"""
        models_dir = MODELS_DIR

        if not os.path.exists(models_dir):
            print(f"Models directory not found: {models_dir}")
            return []
        
        models = []
        
        # If dataset_name is specified, only look in that directory
        if dataset_name:
            dataset_models_dir = os.path.join(models_dir, dataset_name)
            if os.path.exists(dataset_models_dir):
                models.extend(cls._scan_model_directory(dataset_models_dir, dataset_name))
        else:
            # Scan all dataset model directories
            for item in os.listdir(models_dir):
                item_path = os.path.join(models_dir, item)
                if os.path.isdir(item_path):
                    models.extend(cls._scan_model_directory(item_path, item))
        
        # Sort by dataset name and timestamp
        models.sort(key=lambda x: (x['dataset_name'], x['timestamp']))
        return models
    
    @classmethod
    def _scan_model_directory(cls, model_dir, dataset_name):
        """Scan a model directory for .pt files"""
        models = []
        for file in os.listdir(model_dir):
            if file.endswith('.pt') and file != 'latest_model.pt':
                model_path = os.path.join(model_dir, file)
                try:
                    # Load model metadata
                    checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)
                    
                    models.append({
                        'dataset_name': dataset_name,
                        'filename': file,
                        'path': model_path,
                        'timestamp': checkpoint.get('timestamp', 'Unknown'),
                        'epochs': checkpoint.get('epochs', 'Unknown'),
                        'final_train_loss': checkpoint.get('final_train_loss', 'Unknown'),
                        'final_val_loss': checkpoint.get('final_val_loss', 'Unknown'),
                        'batch_size': checkpoint.get('model_config', {}).get('batch_size', 'Unknown'),
                        'learning_rate': checkpoint.get('model_config', {}).get('learning_rate', 'Unknown')
                    })
                except Exception as e:
                    print(f"Failed to load model metadata from {model_path}: {e}")
        
        return models

def main():
    """Main function for model trainer"""
    parser = argparse.ArgumentParser(description='Train DriveGuard neural network')
    parser.add_argument('--dataset-name', help='Specific dataset name to use')
    parser.add_argument('--epochs', type=int, help='Number of training epochs')
    parser.add_argument('--batch-size', type=int, help='Training batch size')
    parser.add_argument('--learning-rate', type=float, help='Learning rate')
    parser.add_argument('--list-datasets', action='store_true', help='List available datasets')
    parser.add_argument('--list-models', action='store_true', help='List available models')
    
    args = parser.parse_args()
    
    if args.list_datasets:
        datasets = ModelTrainer.list_available_datasets()
        if datasets:
            print("Available processed datasets:")
            print("-" * 50)
            for dataset in datasets:
                print(f"  Name: {dataset['name']}")
                print(f"    Samples: {dataset['num_samples']}")
                print(f"    Processed: {dataset['processed_at']}")
                print(f"    Path: {dataset['path']}")
                print()
        else:
            print("No processed datasets found")
        return 0
    
    if args.list_models:
        models = ModelTrainer.list_available_models(args.dataset_name)
        if models:
            print("Available trained models:")
            print("-" * 60)
            current_dataset = None
            for model in models:
                if model['dataset_name'] != current_dataset:
                    current_dataset = model['dataset_name']
                    print(f"\nDataset: {current_dataset}")
                    print("-" * 30)
                
                print(f"  Model: {model['filename']}")
                print(f"    Timestamp: {model['timestamp']}")
                print(f"    Epochs: {model['epochs']}")
                print(f"    Final Train Loss: {model['final_train_loss']}")
                print(f"    Final Val Loss: {model['final_val_loss']}")
                print(f"    Batch Size: {model['batch_size']}")
                print(f"    Learning Rate: {model['learning_rate']}")
                print()
        else:
            if args.dataset_name:
                print(f"No models found for dataset: {args.dataset_name}")
            else:
                print("No trained models found")
        return 0
    
    try:
        trainer = ModelTrainer(args.dataset_name)
        
        # Override parameters if provided
        if args.epochs:
            trainer.num_epochs = args.epochs
            print(f'Overriding epochs to: {args.epochs}')
        
        if args.batch_size:
            trainer.batch_size = args.batch_size
            print(f'Overriding batch size to: {args.batch_size}')
            
        if args.learning_rate:
            trainer.learning_rate = args.learning_rate
            print(f'Overriding learning rate to: {args.learning_rate}')
        
        model_path = trainer.train()
        
        if model_path:
            print(f'Training completed successfully!')
            print(f'Model saved to: {model_path}')
            print(f'Latest model link: {os.path.join(trainer.model_save_dir, "latest_model.pt")}')
        else:
            print('Training failed!')
            return 1
            
    except KeyboardInterrupt:
        print('Training interrupted by user')
        return 1
    except Exception as e:
        print(f'Error: {str(e)}')
        return 1
    
    return 0

if __name__ == '__main__':
    exit(main())