from pathlib import Path
import json
import csv
import cv2
from datetime import datetime
import torch

from ..settings import TOPIC_IMAGE_NAME, TOPIC_CMD_VEL_NAME

# Use ROS2 logger
import rclpy
from rclpy.logging import get_logger

# Initialize ROS2 (if not already initialized)
try:
    rclpy.init()
except RuntimeError:
    pass  # Already initialized

logger = get_logger('vision_control')

class DataRecorder:
    """Use ros2bag to record topics"""

    def __init__(self, topics: list, output_path: str):
        self.topics = topics
        self.output_path = output_path

    def record(self, duration: int = 10):
        """Record topics to ros2bag file"""
        import subprocess
        import time
        import signal
        
        cmd = [
            'ros2', 'bag', 'record',
            '-o', str(self.output_path),
            *self.topics
        ]
        
        logger.info(f"📹 Recording topics: {self.topics} for {duration} seconds")
        
        # Start the recording process
        process = subprocess.Popen(cmd)
        
        try:
            # Wait for the specified duration
            time.sleep(duration)
            
            # Send SIGINT (Ctrl+C) to the process
            process.send_signal(signal.SIGINT)
            
            # Wait for the process to terminate gracefully
            process.wait(timeout=10)
            
        except subprocess.TimeoutExpired:
            # If process doesn't terminate gracefully, force kill it
            process.kill()
            process.wait()
            
        except KeyboardInterrupt:
            # Handle user interruption
            process.send_signal(signal.SIGINT)
            process.wait()
            
        logger.info(f"✅ Recorded data saved to: {self.output_path}")


class VisionControlDatasetGenerator:
    """
    Generate dataset from existing ROS2 bag file
    
    Dataset Directory Structure:
    - `{DATASET_NAME}/`
        - `images/` directory containing all images
        - `data.csv` file with columns: `image_path`, `control_command`
        - `metadata.json` file with metadata about the dataset
    """

    def __init__(self, bag_path: str, out_dir: str, dataset_name: str):
        self.bag_path = Path(bag_path)
        self.out_dir = Path(out_dir)
        self.dataset_name = dataset_name
        
        if not self.bag_path.exists():
            raise ValueError(f"Bag file not found: {self.bag_path}")
        
        # Create output directory structure
        self.dataset_dir = self.out_dir / "datasets" / self.dataset_name
        self.dataset_dir.mkdir(parents=True, exist_ok=True)

    def generate_dataset(self, sync_window_ms: float = 100.0):
        """Generate dataset from ROS2 bag file with message synchronization"""
        try:
            import rosbag2_py
            from rclpy.serialization import deserialize_message
            from sensor_msgs.msg import Image
            from geometry_msgs.msg import Twist
            from cv_bridge import CvBridge
        except ImportError as e:
            logger.error(f"❌ Error importing ROS2 packages: {e}")
            return

        # Create images directory
        images_dir = self.dataset_dir / "images"
        images_dir.mkdir(exist_ok=True)

        bridge = CvBridge()
        dataset_entries = []
        image_count = 0
        
        try:
            # Read bag file
            storage_options = rosbag2_py.StorageOptions(uri=str(self.bag_path), storage_id='sqlite3')
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            reader = rosbag2_py.SequentialReader()
            reader.open(storage_options, converter_options)
            
            topic_types = reader.get_all_topics_and_types()
            type_map = {topic.name: topic.type for topic in topic_types}
            
            # Check if required topics exist
            if TOPIC_IMAGE_NAME not in type_map:
                logger.error(f"❌ Image topic '{TOPIC_IMAGE_NAME}' not found in bag file")
                return
            
            if TOPIC_CMD_VEL_NAME not in type_map:
                logger.error(f"❌ Command velocity topic '{TOPIC_CMD_VEL_NAME}' not found in bag file")
                return
            
            # Collect all messages with timestamps
            image_msgs = []
            cmd_vel_msgs = []
            
            logger.info(f"📦 Processing bag file: {self.bag_path}")
            
            while reader.has_next():
                (topic, data, timestamp) = reader.read_next()
                
                if topic == TOPIC_CMD_VEL_NAME and type_map[topic] == 'geometry_msgs/msg/Twist':
                    msg = deserialize_message(data, Twist)
                    cmd_vel_data = {
                        'linear_x': msg.linear.x,
                        'angular_z': msg.angular.z
                    }
                    cmd_vel_msgs.append((timestamp, cmd_vel_data))
                
                elif topic == TOPIC_IMAGE_NAME and type_map[topic] == 'sensor_msgs/msg/Image':
                    msg = deserialize_message(data, Image)
                    image_msgs.append((timestamp, msg))
            
            # No need to explicitly close reader in newer versions
            del reader
            
            logger.info(f"📊 Found {len(image_msgs)} image messages and {len(cmd_vel_msgs)} cmd_vel messages")
            
            if len(image_msgs) == 0:
                logger.warn("⚠️  No image messages found. Check if the camera was publishing during recording.")
                return
            
            if len(cmd_vel_msgs) == 0:
                logger.warn("⚠️  No cmd_vel messages found. Check if the robot was moving during recording.")
                return
            
            # Synchronize messages
            sync_window_ns = sync_window_ms * 1e6  # Convert to nanoseconds
            
            logger.info(f"🔄 Synchronizing messages with {sync_window_ms}ms window...")
            
            for img_timestamp, img_msg in image_msgs:
                # Find closest cmd_vel message within sync window
                best_cmd_vel = None
                min_time_diff = float('inf')
                
                for cmd_timestamp, cmd_data in cmd_vel_msgs:
                    time_diff = abs(img_timestamp - cmd_timestamp)
                    if time_diff <= sync_window_ns and time_diff < min_time_diff:
                        min_time_diff = time_diff
                        best_cmd_vel = cmd_data
                
                if best_cmd_vel is not None:
                    try:
                        # Convert and save image
                        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
                        image_filename = f"image_{image_count:06d}.jpg"
                        image_path = images_dir / image_filename
                        cv2.imwrite(str(image_path), cv_image)
                        
                        # Store dataset entry
                        dataset_entries.append({
                            'image_path': f"images/{image_filename}",
                            'control_command': json.dumps(best_cmd_vel)
                        })
                        
                        image_count += 1
                        
                        if image_count % 100 == 0:
                            logger.info(f"📸 Processed {image_count} synchronized pairs...")
                            
                    except Exception as e:
                        logger.error(f"❌ Error processing image {image_count}: {e}")
                        continue
            
        except Exception as e:
            logger.error(f"❌ Error reading bag file: {e}")
            return
        
        # Write CSV file
        csv_file = self.dataset_dir / "data.csv"
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['image_path', 'control_command'])
            writer.writeheader()
            for entry in dataset_entries:
                writer.writerow(entry)
        
        # Write metadata file
        metadata = {
            'dataset_name': self.dataset_name,
            'creation_date': datetime.now().isoformat(),
            'recorded_topics': [TOPIC_IMAGE_NAME, TOPIC_CMD_VEL_NAME],
            'bag_file': str(self.bag_path.name),
            'bag_path': str(self.bag_path.absolute()),
            'total_data': image_count
        }
        
        metadata_file = self.dataset_dir / "metadata.json"
        with open(metadata_file, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        success_rate = image_count/len(image_msgs)*100 if image_msgs else 0
        logger.info(f"🎉 Dataset generation completed!")
        logger.info(f"📈 Total synchronized pairs: {image_count}")
        logger.info(f"📊 Success rate: {success_rate:.1f}%")
        logger.info(f"📁 Dataset saved to: {self.dataset_dir}")


class VisionControlDataset(torch.utils.data.Dataset):
    """PyTorch Dataset for Vision Control data"""
    
    def __init__(self, dataset_dir: str):
        self.dataset_dir = Path(dataset_dir)
        self.data_csv = self.dataset_dir / "data.csv"
        
        # Load dataset entries
        self.entries = []
        with open(self.data_csv, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.entries.append(row)
        
        if not self.entries:
            raise ValueError("No data found in the dataset CSV file.")
    
    def __len__(self):
        return len(self.entries)
    
    def __getitem__(self, idx):
        entry = self.entries[idx]
        image_path = self.dataset_dir / entry['image_path']
        image = cv2.imread(str(image_path))
        control_command = json.loads(entry['control_command'])
        
        return {
            'image': image,
            'control_command': control_command
        }
    
    def get_metadata(self):
        metadata_file = self.dataset_dir / "metadata.json"
        with open(metadata_file, 'r') as f:
            return json.load(f)
        generator.generate_dataset(sync_window_ms=sync_window_ms)


class VisionControlDataset(torch.utils.data.Dataset):
    """PyTorch Dataset for Vision Control data"""
    
    def __init__(self, dataset_dir: str):
        self.dataset_dir = Path(dataset_dir)
        self.data_csv = self.dataset_dir / "data.csv"
        
        # Load dataset entries
        self.entries = []
        with open(self.data_csv, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                self.entries.append(row)
        
        if not self.entries:
            raise ValueError("No data found in the dataset CSV file.")
    
    def __len__(self):
        return len(self.entries)
    
    def __getitem__(self, idx):
        entry = self.entries[idx]
        image_path = self.dataset_dir / entry['image_path']
        image = cv2.imread(str(image_path))
        control_command = json.loads(entry['control_command'])
        
        return {
            'image': image,
            'control_command': control_command
        }
    
    def get_metadata(self):
        metadata_file = self.dataset_dir / "metadata.json"
        with open(metadata_file, 'r') as f:
            return json.load(f)
        control_command = json.loads(entry['control_command'])
        
        return {
            'image': image,
            'control_command': control_command
        }
    
    def get_metadata(self):
        metadata_file = self.dataset_dir / "metadata.json"
        with open(metadata_file, 'r') as f:
            return json.load(f)
