from pathlib import Path
import json
import torch
import cv2
import numpy as np
from threading import Lock
import time

# Use ROS2 logger
import rclpy
from rclpy.logging import get_logger

# Initialize ROS2 (if not already initialized)
try:
    rclpy.init()
except RuntimeError:
    pass  # Already initialized

logger = get_logger('vision_control_inferencer')

from .trainer import VisionControlNet
from ..driveguard_node import DriveGuardNode
from geometry_msgs.msg import Twist


class VisionControlInferencer:
    """Real-time vision control inferencer using DriveGuard node"""
    
    def __init__(self, 
                 model_path: str,
                 out_dir: str = None,
                 device: str = "auto",
                 max_linear_speed: float = 0.5,
                 max_angular_speed: float = 1.0):
        
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise ValueError(f"Model file not found: {self.model_path}")
        
        # Device selection
        if device == "auto":
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)
        
        logger.info(f"🔧 Using device: {self.device}")
        
        # Speed limits
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        
        # Load model
        self.model = self._load_model()
        
        # Get DriveGuard node singleton
        self.driveguard_node = DriveGuardNode.get_instance()
        if self.driveguard_node is None:
            raise RuntimeError("DriveGuard node not initialized. Please initialize it first.")
        
        # Inference state
        self.running = False
        self.last_image = None
        self.last_prediction = None
        self.image_lock = Lock()
        
        # Performance tracking
        self.inference_count = 0
        self.total_inference_time = 0.0
        
        logger.info("🤖 Vision Control Inferencer initialized")
        logger.info(f"📦 Model loaded from: {self.model_path}")
        logger.info(f"🚗 Speed limits - Linear: {self.max_linear_speed}, Angular: {self.max_angular_speed}")
    
    def _load_model(self):
        """Load the trained model"""
        try:
            # Load checkpoint
            checkpoint = torch.load(self.model_path, map_location=self.device)
            
            # Create and load model
            model = VisionControlNet().to(self.device)
            model.load_state_dict(checkpoint['model_state_dict'])
            model.eval()
            
            # Log model info
            if 'dataset_metadata' in checkpoint:
                dataset_name = checkpoint['dataset_metadata'].get('dataset_name', 'unknown')
                logger.info(f"📊 Model trained on dataset: {dataset_name}")
            
            if 'val_loss' in checkpoint:
                val_loss = checkpoint['val_loss']
                logger.info(f"🏆 Model validation loss: {val_loss:.4f}")
            
            logger.info("✅ Model loaded successfully")
            return model
            
        except Exception as e:
            logger.error(f"❌ Failed to load model: {e}")
            raise
    
    def _preprocess_image(self, image):
        """Preprocess image for inference (same as training)"""
        if image is None:
            return None
        
        try:
            # Resize image
            image = cv2.resize(image, (224, 224))
            
            # Convert BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Normalize to [0, 1]
            image = image.astype(np.float32) / 255.0
            
            # Convert to tensor and change to CHW format
            image = torch.from_numpy(image).permute(2, 0, 1)
            
            # Add batch dimension
            image = image.unsqueeze(0)
            
            return image.to(self.device)
            
        except Exception as e:
            logger.error(f"❌ Error preprocessing image: {e}")
            return None
    
    def _predict_control(self, image):
        """Make control prediction from image"""
        if image is None:
            return None
        
        try:
            start_time = time.time()
            
            # Preprocess image
            processed_image = self._preprocess_image(image)
            if processed_image is None:
                return None
            
            # Make prediction
            with torch.no_grad():
                prediction = self.model(processed_image)
                linear_x = prediction[0, 0].item()
                angular_z = prediction[0, 1].item()
            
            # Apply speed limits
            linear_x = np.clip(linear_x, -self.max_linear_speed, self.max_linear_speed)
            angular_z = np.clip(angular_z, -self.max_angular_speed, self.max_angular_speed)
            
            # Track performance
            inference_time = time.time() - start_time
            self.inference_count += 1
            self.total_inference_time += inference_time
            
            result = {
                'linear_x': linear_x,
                'angular_z': angular_z,
                'inference_time': inference_time
            }
            
            return result
            
        except Exception as e:
            logger.error(f"❌ Error making prediction: {e}")
            return None
    
    def _get_latest_image(self):
        """Get latest image from driveguard node"""
        try:
            # Use the existing get_raw_image method
            ros_image = self.driveguard_node.get_raw_image()
            if ros_image is None:
                return None
            
            # Convert ROS image to OpenCV format
            import cv_bridge
            bridge = cv_bridge.CvBridge()
            cv_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding='bgr8')
            
            return cv_image
            
        except Exception as e:
            logger.error(f"❌ Error getting image: {e}")
            return None
    
    def _publish_control_command(self, linear_x: float, angular_z: float) -> bool:
        """Publish control command using driveguard node"""
        try:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            
            # Use the existing set_twist method
            self.driveguard_node.set_twist(twist)
            return True
            
        except Exception as e:
            logger.error(f"❌ Error publishing control command: {e}")
            return False
    
    def start_inference(self, publish_rate: float = 10.0):
        """Start real-time inference loop"""
        if self.running:
            logger.warn("⚠️  Inference already running")
            return
        
        logger.info(f"🚀 Starting vision control inference at {publish_rate} Hz")
        
        self.running = True
        
        # Inference loop
        rate_sleep = 1.0 / publish_rate
        
        try:
            while self.running:
                start_loop_time = time.time()
                
                # Get latest image from driveguard node
                current_image = self._get_latest_image()
                
                if current_image is not None:
                    # Make prediction
                    prediction = self._predict_control(current_image)
                    
                    if prediction is not None:
                        # Update last prediction
                        self.last_prediction = prediction
                        
                        # Publish control command
                        success = self._publish_control_command(
                            linear_x=prediction['linear_x'],
                            angular_z=prediction['angular_z']
                        )
                        
                        if not success:
                            logger.warn("⚠️  Failed to publish control command")
                        
                        # Log performance periodically
                        if self.inference_count % 100 == 0:
                            avg_inference_time = self.total_inference_time / self.inference_count
                            logger.info(f"📊 Inference stats - Count: {self.inference_count}, "
                                      f"Avg time: {avg_inference_time*1000:.2f}ms")
                    else:
                        # Send stop command if prediction failed
                        self._publish_control_command(linear_x=0.0, angular_z=0.0)
                else:
                    # Send stop command if no image available
                    self._publish_control_command(linear_x=0.0, angular_z=0.0)
                
                # Maintain publish rate
                loop_time = time.time() - start_loop_time
                sleep_time = max(0, rate_sleep - loop_time)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            logger.info("🛑 Inference stopped by user")
        except Exception as e:
            logger.error(f"❌ Error in inference loop: {e}")
        finally:
            self.stop_inference()
    
    def stop_inference(self):
        """Stop the inference loop"""
        if not self.running:
            return
        
        logger.info("🛑 Stopping vision control inference")
        
        self.running = False
        
        # Send stop command
        self._publish_control_command(linear_x=0.0, angular_z=0.0)
        
        # Log final stats
        if self.inference_count > 0:
            avg_inference_time = self.total_inference_time / self.inference_count
            logger.info(f"📊 Final inference stats:")
            logger.info(f"  Total inferences: {self.inference_count}")
            logger.info(f"  Average inference time: {avg_inference_time*1000:.2f}ms")
            logger.info(f"  Average FPS: {1.0/avg_inference_time:.1f}")
        
        logger.info("✅ Vision control inference stopped")
    
    def get_status(self):
        """Get current inferencer status"""
        # Check if image is available
        has_image = self._get_latest_image() is not None
        
        status = {
            'running': self.running,
            'has_image': has_image,
            'inference_count': self.inference_count,
            'last_prediction': self.last_prediction,
            'model_path': str(self.model_path),
            'device': str(self.device),
            'speed_limits': {
                'max_linear_speed': self.max_linear_speed,
                'max_angular_speed': self.max_angular_speed
            }
        }
        
        if self.inference_count > 0:
            status['avg_inference_time'] = self.total_inference_time / self.inference_count
            status['avg_fps'] = 1.0 / status['avg_inference_time']
        
        return status
    
    def predict_single_image(self, image_path: str):
        """Make prediction on a single image file (for testing)"""
        try:
            # Load image
            image = cv2.imread(image_path)
            if image is None:
                raise ValueError(f"Could not load image: {image_path}")
            
            # Make prediction
            prediction = self._predict_control(image)
            
            if prediction is not None:
                logger.info(f"🔮 Single image prediction:")
                logger.info(f"  Image: {image_path}")
                logger.info(f"  Linear velocity: {prediction['linear_x']:.4f}")
                logger.info(f"  Angular velocity: {prediction['angular_z']:.4f}")
                logger.info(f"  Inference time: {prediction['inference_time']*1000:.2f}ms")
            
            return prediction
            
        except Exception as e:
            logger.error(f"❌ Error in single image prediction: {e}")
            return None


def create_inferencer_from_model_name(model_name: str, out_dir: str = None, **kwargs):
    """Create inferencer from model name"""
    import os
    
    if out_dir is None:
        workspace_dir = os.getenv('WORKSPACE_DIR', '.')
        out_dir = workspace_dir + "/" + "out"
    
    out_dir = Path(out_dir)
    model_path = out_dir / "models" / model_name / "best_model.pth"
    
    if not model_path.exists():
        raise ValueError(f"Model not found: {model_path}")
    
    return VisionControlInferencer(str(model_path), str(out_dir), **kwargs)
