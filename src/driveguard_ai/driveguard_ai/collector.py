import os
import numpy as np
import cv2
from cv_bridge import CvBridge
import torch
from torch.utils.data import Dataset
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
from datetime import datetime
import yaml
import argparse
import subprocess
import signal
import time

class DriveGuardDataset(Dataset):
    """PyTorch dataset for DriveGuard images and commands"""
    
    def __init__(self, images, commands, image_size=(224, 224), transform=None):
        self.images = images
        self.commands = commands
        self.image_size = image_size
        self.transform = transform
        self.cv_bridge = CvBridge()
    
    def __len__(self):
        return len(self.images)
    
    def __getitem__(self, idx):
        # Convert ROS Image to CV2 image and then to tensor
        cv_image = self.cv_bridge.imgmsg_to_cv2(self.images[idx], desired_encoding='rgb8')
        
        # Resize to specified size
        cv_image = cv2.resize(cv_image, self.image_size)
        
        # Convert to tensor
        image_tensor = torch.from_numpy(cv_image.transpose((2, 0, 1))).float() / 255.0
        
        if self.transform:
            image_tensor = self.transform(image_tensor)
        
        # Convert command to tensor
        command = self.commands[idx]
        command_tensor = torch.tensor([command.linear.x, command.angular.z], dtype=torch.float)
        
        return image_tensor, command_tensor


class DataRecorder:
    """Rosbag recorder that saves to organized dataset directories"""
    
    def __init__(self, config_path=None):
        # Load configuration
        if not config_path:
            config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                    'config', 'driveguard_config.yaml')
        
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            self.data_config = config['data_collection']
        
        # Setup directories and names
        self.datasets_dir = self.data_config['datasets_dir']
        if not self.datasets_dir:
            self.datasets_dir = "./out/datasets"
        self.datasets_dir = os.path.expanduser(self.datasets_dir)
        
        self.dataset_name = self.data_config['dataset_name']
        if not self.dataset_name:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.dataset_name = f"dataset_{timestamp}"
        
        # Recording parameters
        self.topics_to_record = self.data_config['topics_to_record']
        self.recording_duration = self.data_config['recording_duration']
        self.max_bag_size = self.data_config['max_bag_size']
        
        
        self.recording_process = None
    
    def start_recording(self):
        """Start recording to the dataset directory"""
        self.dataset_path = os.path.join(self.datasets_dir, self.dataset_name)
        os.makedirs(self.dataset_path, exist_ok=True)
        print(f"Dataset directory: {self.dataset_path}")
        
        bag_name = "rosbag"
        bag_path = os.path.join(self.dataset_path, bag_name)
        
        cmd = [
            'ros2', 'bag', 'record',
            '--output', bag_path,
            '--max-bag-size', str(self.max_bag_size * 1024 * 1024)
        ] + self.topics_to_record
        
        print(f'Starting recording: {" ".join(cmd)}')
        print(f'Bag will be saved to: {bag_path}')
        
        try:
            self.recording_process = subprocess.Popen(cmd)
            print(f'Recording started, PID: {self.recording_process.pid}')
            
            # Wait for completion or duration timeout
            if self.recording_duration > 0:
                try:
                    self.recording_process.wait(timeout=self.recording_duration)
                    print('Recording completed successfully')
                except subprocess.TimeoutExpired:
                    print('Recording duration reached, stopping...')
                    self.stop_recording()
            else:
                # Wait indefinitely
                print('Recording indefinitely (Ctrl+C to stop)...')
                self.recording_process.wait()
                
        except Exception as e:
            print(f'Failed to start recording: {str(e)}')
            raise
    
    def stop_recording(self):
        """Stop recording"""
        if self.recording_process:
            print('Stopping recording...')
            self.recording_process.send_signal(signal.SIGINT)
            try:
                self.recording_process.wait(timeout=10)
                print('Recording stopped successfully')
            except subprocess.TimeoutExpired:
                print('Force killing recording process...')
                self.recording_process.kill()
    
    def get_dataset_path(self):
        """Return the dataset path where bag and processed data will be stored"""
        return self.dataset_path


class DataProcessor:
    """Data processor for processing rosbag files in dataset directories"""
    def __init__(self, config_path=None, dataset_name=None):
        # Load configuration from YAML
        if not config_path:
            config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                    'config', 'driveguard_config.yaml')
        
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                self.data_config = config['data_collection']
                self.training_config = config['training']
        except Exception as e:
            print(f'Failed to load config: {str(e)}')
            raise
        
        # Setup directories
        self.datasets_dir = self.data_config['datasets_dir']
        if not self.datasets_dir:
            self.datasets_dir = "./out/datasets"
        self.datasets_dir = os.path.expanduser(self.datasets_dir)
        
        # Determine dataset name
        if dataset_name:
            self.dataset_name = dataset_name
        elif self.data_config['dataset_name']:
            self.dataset_name = self.data_config['dataset_name']
        else:
            # Find the latest dataset directory
            self.dataset_name = self._find_latest_dataset()
            if not self.dataset_name:
                raise FileNotFoundError('No dataset found. Please specify dataset_name or record data first.')
        
        self.dataset_path = os.path.join(self.datasets_dir, self.dataset_name)
        
        if not os.path.exists(self.dataset_path):
            raise FileNotFoundError(f'Dataset directory not found: {self.dataset_path}')
        
        # Find rosbag in dataset directory
        self.bag_path = os.path.join(self.dataset_path, "rosbag")
        if not os.path.exists(self.bag_path) or not os.path.exists(os.path.join(self.bag_path, 'metadata.yaml')):
            raise FileNotFoundError(f'Rosbag not found in dataset: {self.bag_path}')
        
        # Extract image size from training config
        self.image_size = tuple(self.training_config['image_size'])
        
        print(f'Data collector initialized with:')
        print(f'  Dataset: {self.dataset_name}')
        print(f'  Dataset path: {self.dataset_path}')
        print(f'  Bag path: {self.bag_path}')
        print(f'  Image size: {self.image_size}')
    
    def _find_latest_dataset(self):
        """Find the latest dataset directory"""
        if not os.path.exists(self.datasets_dir):
            return None
            
        dataset_dirs = []
        for item in os.listdir(self.datasets_dir):
            item_path = os.path.join(self.datasets_dir, item)
            if os.path.isdir(item_path):
                # Check if it contains a rosbag
                bag_path = os.path.join(item_path, "rosbag")
                if os.path.exists(os.path.join(bag_path, 'metadata.yaml')):
                    dataset_dirs.append(item)
        
        if not dataset_dirs:
            return None
            
        # Return the latest dataset based on name (should contain timestamp)
        latest_dataset = max(dataset_dirs)
        return latest_dataset
    
    def collect_data(self):
        """Extract images and commands from the rosbag"""
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        
        print(f"Reading from bag: {self.bag_path}")
        
        # Get topics from config
        image_topic = self.data_config['topics_to_record'][0]  # Assume first is image
        cmd_topic = self.data_config['topics_to_record'][1]    # Assume second is cmd_vel
        
        print(f"Looking for topics: {image_topic} and {cmd_topic}")
        
        storage_options = StorageOptions(uri=self.bag_path, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr')
        
        reader = SequentialReader()
        reader.open(storage_options, converter_options)
        
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
        
        # Check if required topics exist in the bag
        if image_topic not in type_map:
            print(f"Error: Image topic '{image_topic}' not found in bag file")
            print(f"Available topics: {list(type_map.keys())}")
            return [], []
            
        if cmd_topic not in type_map:
            print(f"Error: Command topic '{cmd_topic}' not found in bag file")
            print(f"Available topics: {list(type_map.keys())}")
            return [], []
        
        images = []
        commands = []
        image_timestamps = []
        cmd_timestamps = []
        
        # Process messages
        message_count = 0
        while reader.has_next():
            topic_name, data, timestamp = reader.read_next()
            message_count += 1
            
            if message_count % 1000 == 0:
                print(f"Processed {message_count} messages...")
            
            if topic_name == image_topic:
                msg = deserialize_message(data, type_map[topic_name])
                images.append(msg)
                image_timestamps.append(timestamp)
            
            elif topic_name == cmd_topic:
                msg = deserialize_message(data, type_map[topic_name])
                commands.append(msg)
                cmd_timestamps.append(timestamp)
        
        print(f"Collected {len(images)} images and {len(commands)} commands")
        
        # Synchronize images and commands based on timestamps
        synced_images, synced_commands = self._synchronize_data(
            images, commands, image_timestamps, cmd_timestamps)
        
        print(f"After synchronization: {len(synced_images)} paired samples")
        return synced_images, synced_commands
    
    def _synchronize_data(self, images, commands, image_timestamps, cmd_timestamps):
        """Synchronize images and commands based on closest timestamps"""
        if not images or not commands:
            return [], []
        
        synced_images = []
        synced_commands = []
        
        for i, img_time in enumerate(image_timestamps):
            # Find the command with the closest timestamp
            closest_cmd_idx = min(range(len(cmd_timestamps)), 
                               key=lambda j: abs(cmd_timestamps[j] - img_time))
            
            # Check time difference (optional: add threshold)
            time_diff = abs(cmd_timestamps[closest_cmd_idx] - img_time) / 1e9  # Convert to seconds
            if time_diff > 0.1:  # Skip if time difference > 100ms
                continue
                
            synced_images.append(images[i])
            synced_commands.append(commands[closest_cmd_idx])
        
        return synced_images, synced_commands
    
    def save_dataset(self, images, commands):
        """Save the processed dataset in the same directory as the rosbag"""
        if not images or not commands:
            print("Warning: No data to save")
            return None
        
        # Create PyTorch dataset
        dataset = DriveGuardDataset(images, commands, self.image_size)
        
        # Save as PyTorch dataset
        dataset_file = os.path.join(self.dataset_path, 'processed_dataset.pt')
        torch.save({
            'images': images,
            'commands': commands,
            'image_size': self.image_size
        }, dataset_file)
        
        # Save metadata
        metadata = {
            'num_samples': len(images),
            'image_topic': self.data_config['topics_to_record'][0],
            'cmd_topic': self.data_config['topics_to_record'][1],
            'image_size': self.image_size,
            'source_bag': self.bag_path,
            'processed_at': datetime.now().strftime("%Y%m%d_%H%M%S")
        }
        
        metadata_file = os.path.join(self.dataset_path, 'dataset_metadata.yaml')
        with open(metadata_file, 'w') as f:
            yaml.dump(metadata, f)
        
        # Save sample images for visualization
        self._save_samples(images, commands)
        
        print(f"Processed dataset saved to: {self.dataset_path}")
        print(f"  - Dataset file: {dataset_file}")
        print(f"  - Metadata: {metadata_file}")
        return self.dataset_path
    
    def _save_samples(self, images, commands, num_samples=5):
        """Save sample images with corresponding commands for visualization"""
        cv_bridge = CvBridge()
        samples_dir = os.path.join(self.dataset_path, 'samples')
        os.makedirs(samples_dir, exist_ok=True)
        
        if len(images) < num_samples:
            num_samples = len(images)
            
        indices = np.linspace(0, len(images) - 1, num_samples, dtype=int)
        
        for i, idx in enumerate(indices):
            image = images[idx]
            command = commands[idx]
            
            try:
                cv_image = cv_bridge.imgmsg_to_cv2(image, desired_encoding='rgb8')
                cv_image = cv2.resize(cv_image, self.image_size)
                
                plt.figure(figsize=(10, 6))
                plt.imshow(cv_image)
                plt.title(f"Linear: {command.linear.x:.2f}, Angular: {command.angular.z:.2f}")
                plt.axis('off')
                plt.savefig(os.path.join(samples_dir, f"sample_{i}.png"), bbox_inches='tight')
                plt.close()
            except Exception as e:
                print(f"Warning: Failed to save sample {i}: {e}")

    def process_dataset(self):
        """Process the rosbag and save the dataset"""
        print('Starting data processing...')
        try:
            images, commands = self.collect_data()
            if images and commands:
                dataset_path = self.save_dataset(images, commands)
                print(f'Success! Processed dataset saved to {dataset_path}')
                return dataset_path
            else:
                print('Warning: No data collected')
                return None
        except Exception as e:
            print(f'Error processing data: {str(e)}')
            raise

    @classmethod
    def list_datasets(cls, config_path=None):
        """List all available datasets"""
        if not config_path:
            config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                                    'config', 'driveguard_config.yaml')
        
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
            datasets_dir = config['data_collection']['datasets_dir']
            if not datasets_dir:
                datasets_dir = "./out/datasets"
            datasets_dir = os.path.expanduser(datasets_dir)
        
        if not os.path.exists(datasets_dir):
            print(f"Datasets directory not found: {datasets_dir}")
            return []
        
        datasets = []
        for item in os.listdir(datasets_dir):
            item_path = os.path.join(datasets_dir, item)
            if os.path.isdir(item_path):
                bag_path = os.path.join(item_path, "rosbag")
                if os.path.exists(os.path.join(bag_path, 'metadata.yaml')):
                    # Check if processed dataset exists
                    processed = os.path.exists(os.path.join(item_path, 'processed_dataset.pt'))
                    datasets.append({
                        'name': item,
                        'path': item_path,
                        'processed': processed
                    })
        
        return datasets


def deserialize_message(data, topic_type):
    """Deserialize binary message data based on the topic type"""
    from rclpy.serialization import deserialize_message as rclpy_deserialize
    
    if topic_type == 'sensor_msgs/msg/Image':
        from sensor_msgs.msg import Image
        return rclpy_deserialize(data, Image)
    elif topic_type == 'geometry_msgs/msg/Twist':
        from geometry_msgs.msg import Twist
        return rclpy_deserialize(data, Twist)
    else:
        raise NotImplementedError(f"Deserialization for {topic_type} not implemented")


def record_main():
    """Main function for recording rosbags"""
    parser = argparse.ArgumentParser(description='Record ROS2 topics to organized dataset')
    parser.add_argument('--config', help='Config file path')
    parser.add_argument('--dataset-name', help='Dataset name (will create directory)')
    parser.add_argument('--duration', type=int, help='Recording duration in seconds')
    
    args = parser.parse_args()
    
    try:
        recorder = DataRecorder(args.config)
        
        # Override dataset name if provided
        if args.dataset_name:
            recorder.dataset_name = args.dataset_name
            recorder.dataset_path = os.path.join(recorder.datasets_dir, recorder.dataset_name)
            os.makedirs(recorder.dataset_path, exist_ok=True)
        
        # Override duration if provided
        if args.duration:
            recorder.recording_duration = args.duration
            
        print(f"Recording to dataset: {recorder.dataset_name}")
        recorder.start_recording()
        
    except KeyboardInterrupt:
        print('\nRecording interrupted by user')
        if 'recorder' in locals():
            recorder.stop_recording()
    except Exception as e:
        print(f'Error: {str(e)}')


def process_main():
    """Main function for processing datasets"""
    parser = argparse.ArgumentParser(description='Process rosbag data into training dataset')
    parser.add_argument('--config', help='Config file path')
    parser.add_argument('--dataset-name', help='Dataset name to process')
    parser.add_argument('--list', action='store_true', help='List available datasets')
    
    args = parser.parse_args()
    
    if args.list:
        datasets = DataProcessor.list_datasets(args.config)
        if datasets:
            print("Available datasets:")
            for dataset in datasets:
                status = "✓ processed" if dataset['processed'] else "○ raw only"
                print(f"  {dataset['name']} - {status}")
        else:
            print("No datasets found")
        return
    
    try:
        collector = DataProcessor(args.config, args.dataset_name)
        collector.process_dataset()
        
    except KeyboardInterrupt:
        print('\nProcess interrupted by user')
    except Exception as e:
        print(f'Error: {str(e)}')


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == "record":
        sys.argv.pop(1)  # Remove 'record' from args
        record_main()
    elif len(sys.argv) > 1 and sys.argv[1] == "process":
        sys.argv.pop(1)  # Remove 'process' from args
        process_main()
    else:
        print("Usage:")
        print("  python data_collection.py record [options]  - Record rosbag data")
        print("  python data_collection.py process [options] - Process rosbag into dataset")
        print("  python data_collection.py process --list    - List available datasets")



