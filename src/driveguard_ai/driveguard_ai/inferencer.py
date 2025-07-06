import os
import torch
import cv2
import numpy as np
from cv_bridge import CvBridge
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from driveguard_ai.models.resnet import DriveGuardResNet18

class InferenceNode(Node):
    def __init__(self):
        super().__init__('driveguard_inference')
        
        # Load configuration from YAML
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').value
        if not config_path:
            config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                    'config', 'driveguard_config.yaml')
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                infer_config = config['inference']
                self.global_config = config
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {str(e)}')
            return
        
        # Parameters from config using global paths
        self.model_dir = os.path.expanduser(self.global_config['model_dir'])
        self.image_topic = infer_config['image_topic']
        self.cmd_topic = infer_config['cmd_topic']
        
        # Find the latest model
        self.model_path = self._find_latest_model()
        
        if not self.model_path:
            self.get_logger().error('No model files found')
            return
        
        # Determine device
        if infer_config['device'] == 'auto':
            self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        else:
            self.device = torch.device(infer_config['device'])
        
        # Publishers and subscribers
        self.cmd_publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        
        # CV Bridge
        self.cv_bridge = CvBridge()
        
        # Load model
        self.model = None
        self.load_model()
        
        self.get_logger().info(f'DriveGuard Inference Node started, subscribing to {self.image_topic}, publishing to {self.cmd_topic}')
    
    def _find_latest_model(self):
        """Find the latest model file in the model directory"""
        if not os.path.exists(self.model_dir):
            self.get_logger().error(f'Model directory does not exist: {self.model_dir}')
            return None
            
        # Look for model files
        model_files = []
        for item in os.listdir(self.model_dir):
            if item.endswith('.pt') and item.startswith('driveguard_model_'):
                model_files.append(os.path.join(self.model_dir, item))
        
        if not model_files:
            self.get_logger().error(f'No model files found in {self.model_dir}')
            return None
            
        # Return the latest model file based on name (timestamp)
        latest_model = max(model_files, key=lambda x: os.path.basename(x))
        self.get_logger().info(f'Using latest model: {latest_model}')
        return latest_model
    
    def load_model(self):
        """Load the trained PyTorch model"""
        if not self.model_path or not os.path.exists(self.model_path):
            self.get_logger().error(f'Model path does not exist: {self.model_path}')
            return False
        
        try:
            # Initialize model
            self.model = DriveGuardResNet18(num_outputs=2)
            
            # Load weights
            checkpoint = torch.load(self.model_path, map_location=self.device)
            self.model.load_state_dict(checkpoint['model_state_dict'])
            
            # Set to evaluation mode
            self.model.to(self.device)
            self.model.eval()
            
            self.get_logger().info(f'Model loaded from {self.model_path}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error loading model: {str(e)}')
            return False
    
    def image_callback(self, msg):
        """Process incoming image messages and publish control commands"""
        if self.model is None:
            self.get_logger().warn('No model loaded, skipping inference')
            return
        
        try:
            # Convert ROS Image to CV2 image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Preprocess image
            cv_image = cv2.resize(cv_image, (224, 224))
            
            # Convert to PyTorch tensor
            image_tensor = torch.from_numpy(cv_image.transpose((2, 0, 1))).float() / 255.0
            image_tensor = image_tensor.unsqueeze(0)  # Add batch dimension
            image_tensor = image_tensor.to(self.device)
            
            # Inference
            with torch.no_grad():
                output = self.model(image_tensor)
            
            # Extract predictions
            linear_velocity = float(output[0][0])
            angular_velocity = float(output[0][1])
            
            # Create and publish Twist message
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity
            
            self.cmd_publisher.publish(twist_msg)
            
            # Log periodically
            self.get_logger().debug(f'Published cmd_vel: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error during inference: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    inference_node = InferenceNode()
    
    try:
        rclpy.spin(inference_node)
    except KeyboardInterrupt:
        inference_node.get_logger().info('Keyboard interrupt, shutting down')
    finally:
        inference_node.destroy_node()

if __name__ == '__main__':
    main()