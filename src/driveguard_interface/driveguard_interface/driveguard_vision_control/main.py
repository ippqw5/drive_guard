from .collector import DataRecorder, VisionControlDatasetGenerator
from .trainer import VisionControlTrainer
from .inferencer import VisionControlInferencer, create_inferencer_from_model_name
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

"""
usage:

ros2 run driveguard_interface vision_control record --duration 10 [--out_dir <OUT_DIR>] [--bag_name <BAG_NAME>]
ros2 run driveguard_interface vision_control generate --name <dataset_name> [--bag <path>] [--out_dir <OUT_DIR>]
ros2 run driveguard_interface vision_control list [--out_dir <OUT_DIR>]
ros2 run driveguard_interface vision_control train --dataset <dataset_name> [--model_name <model_name>] [--epochs 100] [--batch_size 32] [--learning_rate 0.001] [--out_dir <OUT_DIR>]
ros2 run driveguard_interface vision_control evaluate --model <model_name> --dataset <dataset_name> [--out_dir <OUT_DIR>]
ros2 run driveguard_interface vision_control inference --model <model_name> [--rate 10.0] [--max_linear 0.5] [--max_angular 1.0] [--device <DEVICE>] [--out_dir <OUT_DIR>]
ros2 run driveguard_interface vision_control test --model <model_name> --image <image_path> [--out_dir <OUT_DIR>]

"""
def main():
    import argparse
    from pathlib import Path
    from datetime import datetime
    import os
    
    # Default output directory
    workspace_dir = os.getenv('WORKSPACE_DIR', '.')
    default_out_dir = workspace_dir + "/" + "out"

    parser = argparse.ArgumentParser(description="Vision Control Data Collector, Trainer and Inferencer")
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Record command
    record_parser = subparsers.add_parser('record', help='Record ros2bag data')
    record_parser.add_argument('--duration', type=float, default=10.0, help='Duration for recording in seconds (default: 10.0)')
    record_parser.add_argument('--out_dir', type=str, default=default_out_dir, help=f'Output directory (default: {default_out_dir})')
    record_parser.add_argument('--bag_name', type=str, help='Custom bag name (default: timestamped name)')
    
    # Generate command
    generate_parser = subparsers.add_parser('generate', help='Generate dataset from ros2bag')
    generate_parser.add_argument('--bag', type=str, help='Path to ros2bag file (optional - will show interactive selection if not provided)')
    generate_parser.add_argument('--name', type=str, required=True, help='Name of the dataset')
    generate_parser.add_argument('--out_dir', type=str, default=default_out_dir, help=f'Output directory (default: {default_out_dir})')
    
    # List command
    list_parser = subparsers.add_parser('list', help='List available bag files, datasets, and models')
    list_parser.add_argument('--out_dir', type=str, default=default_out_dir, help=f'Output directory (default: {default_out_dir})')
    
    # Train command
    train_parser = subparsers.add_parser('train', help='Train vision control model')
    train_parser.add_argument('--dataset', type=str, help='Dataset name (optional - will show interactive selection if not provided)')
    train_parser.add_argument('--model_name', type=str, default='vision_control_model', help='Model name (default: vision_control_model)')
    train_parser.add_argument('--epochs', type=int, default=100, help='Number of epochs (default: 100)')
    train_parser.add_argument('--batch_size', type=int, default=32, help='Batch size (default: 32)')
    train_parser.add_argument('--learning_rate', type=float, default=0.001, help='Learning rate (default: 0.001)')
    train_parser.add_argument('--val_split', type=float, default=0.2, help='Validation split ratio (default: 0.2)')
    train_parser.add_argument('--device', type=str, default='auto', help='Device to use (default: auto)')
    train_parser.add_argument('--patience', type=int, default=50, help='Early stopping patience (default: 50)')
    train_parser.add_argument('--out_dir', type=str, default=default_out_dir, help=f'Output directory (default: {default_out_dir})')
    
    # Evaluate command
    evaluate_parser = subparsers.add_parser('evaluate', help='Evaluate trained model')
    evaluate_parser.add_argument('--model', type=str, help='Model name (optional - will show interactive selection if not provided)')
    evaluate_parser.add_argument('--dataset', type=str, help='Dataset name (optional - will show interactive selection if not provided)')
    evaluate_parser.add_argument('--out_dir', type=str, default=default_out_dir, help=f'Output directory (default: {default_out_dir})')
    
    # Inference command
    inference_parser = subparsers.add_parser('inference', help='Run real-time vision control inference')
    inference_parser.add_argument('--model', type=str, help='Model name (optional - will show interactive selection if not provided)')
    inference_parser.add_argument('--rate', type=float, default=10.0, help='Publish rate in Hz (default: 10.0)')
    inference_parser.add_argument('--max_linear', type=float, default=0.5, help='Max linear speed (default: 0.5)')
    inference_parser.add_argument('--max_angular', type=float, default=1.0, help='Max angular speed (default: 1.0)')
    inference_parser.add_argument('--device', type=str, default='auto', help='Device to use (default: auto)')
    inference_parser.add_argument('--out_dir', type=str, default=default_out_dir, help=f'Output directory (default: {default_out_dir})')
    
    # Test prediction command
    test_parser = subparsers.add_parser('test', help='Test prediction on single image')
    test_parser.add_argument('--model', type=str, help='Model name (optional - will show interactive selection if not provided)')
    test_parser.add_argument('--image', type=str, required=True, help='Path to image file')
    test_parser.add_argument('--out_dir', type=str, default=default_out_dir, help=f'Output directory (default: {default_out_dir})')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    # Check if using default directory and show bold reminder
    if args.out_dir == default_out_dir:
        print(f"\033[1m📁 Using default output directory: {default_out_dir}\033[0m")
    
    if args.command == 'record':
        # Create output directory structure
        out_dir = Path(args.out_dir)
        ros2bags_dir = out_dir / "ros2bags"
        ros2bags_dir.mkdir(parents=True, exist_ok=True)
        
        # Generate bag name
        if args.bag_name:
            bag_name = args.bag_name
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = f"rosbag2_{timestamp}"
        
        bag_file = ros2bags_dir / bag_name
        
        # Record data
        topics = [TOPIC_IMAGE_NAME, TOPIC_CMD_VEL_NAME]
        recorder = DataRecorder(topics=topics, output_path=str(bag_file))
        recorder.record(duration=int(args.duration))
        logger.info(f"📦 Bag file created: {bag_file}")
        
    elif args.command == 'generate':
        # If bag path is not provided, show interactive selection
        if not args.bag:
            out_dir = Path(args.out_dir)
            ros2bags_dir = out_dir / "ros2bags"
            
            if not ros2bags_dir.exists():
                logger.error(f"❌ No ros2bags directory found in {out_dir}")
                return
            
            # Find all bag files
            bag_files = []
            for item in ros2bags_dir.iterdir():
                if item.is_dir():
                    # Check for both rosbag2_ prefix and custom names
                    metadata_file = item / 'metadata.yaml'
                    if metadata_file.exists():
                        bag_files.append(item)
            
            if not bag_files:
                logger.error(f"❌ No bag files found in {ros2bags_dir}")
                return
            
            # Show interactive selection
            print("📋 Available bag files:")
            for i, bag_file in enumerate(sorted(bag_files), 1):
                print(f"  {i:2d}. {bag_file.name}")
            
            try:
                choice = input("\n🎯 Select a bag file (enter number): ").strip()
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(bag_files):
                    selected_bag = sorted(bag_files)[choice_idx]
                    logger.info(f"✅ Selected: {selected_bag.name}")
                else:
                    logger.error("❌ Invalid selection")
                    return
            except (ValueError, KeyboardInterrupt):
                logger.warn("⚠️  Selection cancelled")
                return
            
            bag_path = str(selected_bag)
        else:
            bag_path = args.bag
        
        generator = VisionControlDatasetGenerator(
            bag_path=bag_path,
            out_dir=args.out_dir,
            dataset_name=args.name
        )
        generator.generate_dataset()
        
    elif args.command == 'list':
        out_dir = Path(args.out_dir)
        if not out_dir.exists():
            logger.error(f"❌ Output directory {out_dir} does not exist")
            return
        
        # Find all bag files in ros2bags directory
        ros2bags_dir = out_dir / "ros2bags"
        bag_files = []
        if ros2bags_dir.exists():
            for item in ros2bags_dir.iterdir():
                if item.is_dir():
                    # Check if it's a valid bag directory (remove rosbag2_ prefix requirement)
                    metadata_file = item / 'metadata.yaml'
                    if metadata_file.exists():
                        bag_files.append(item)
        
        if bag_files:
            print(f"📦 Available bag files in {ros2bags_dir}:")
            for i, bag_file in enumerate(sorted(bag_files), 1):
                print(f"  {i:2d}. {bag_file.name}")
        else:
            print(f"📦 No bag files found in {ros2bags_dir}")
        
        # Show datasets in datasets directory
        datasets_dir = out_dir / "datasets"
        datasets = []
        if datasets_dir.exists():
            for item in datasets_dir.iterdir():
                if item.is_dir():
                    metadata_file = item / 'metadata.json'
                    if metadata_file.exists():
                        datasets.append(item)
        
        if datasets:
            print(f"\n📊 Existing datasets in {datasets_dir}:")
            for i, dataset in enumerate(sorted(datasets), 1):
                print(f"  {i:2d}. {dataset.name}")
        else:
            print(f"\n📊 No datasets found in {datasets_dir}")
        
        # Show models in models directory
        models_dir = out_dir / "models"
        models = []
        if models_dir.exists():
            for item in models_dir.iterdir():
                if item.is_dir():
                    # Check if it's a valid model directory
                    best_model_file = item / 'best_model.pth'
                    metadata_file = item / 'metadata.json'
                    if best_model_file.exists() and metadata_file.exists():
                        models.append(item)
        
        if models:
            print(f"\n🤖 Available models in {models_dir}:")
            for i, model in enumerate(sorted(models), 1):
                print(f"  {i:2d}. {model.name}")
        else:
            print(f"\n🤖 No models found in {models_dir}")
    
    elif args.command == 'train':
        # Interactive dataset selection if not provided
        if not args.dataset:
            out_dir = Path(args.out_dir)
            datasets_dir = out_dir / "datasets"
            
            if not datasets_dir.exists():
                logger.error(f"❌ No datasets directory found in {out_dir}")
                return
            
            # Find all datasets
            datasets = []
            for item in datasets_dir.iterdir():
                if item.is_dir():
                    metadata_file = item / 'metadata.json'
                    if metadata_file.exists():
                        datasets.append(item)
            
            if not datasets:
                logger.error(f"❌ No datasets found in {datasets_dir}")
                return
            
            # Show interactive selection
            print("📊 Available datasets:")
            for i, dataset in enumerate(sorted(datasets), 1):
                print(f"  {i:2d}. {dataset.name}")
            
            try:
                choice = input("\n🎯 Select a dataset (enter number): ").strip()
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(datasets):
                    selected_dataset = sorted(datasets)[choice_idx]
                    logger.info(f"✅ Selected dataset: {selected_dataset.name}")
                    dataset_path = str(selected_dataset)
                else:
                    logger.error("❌ Invalid selection")
                    return
            except (ValueError, KeyboardInterrupt):
                logger.warn("⚠️  Selection cancelled")
                return
        else:
            dataset_path = str(Path(args.out_dir) / "datasets" / args.dataset)
        
        # Create trainer and start training
        try:
            trainer = VisionControlTrainer(
                dataset_dir=dataset_path,
                model_name=args.model_name,
                out_dir=args.out_dir,
                device=args.device
            )
            
            trainer.train(
                epochs=args.epochs,
                batch_size=args.batch_size,
                learning_rate=args.learning_rate,
                val_split=args.val_split,
                patience=args.patience
            )
            
        except Exception as e:
            logger.error(f"❌ Training failed: {e}")
    
    elif args.command == 'evaluate':
        # Interactive model selection if not provided
        if not args.model:
            out_dir = Path(args.out_dir)
            models_dir = out_dir / "models"
            
            if not models_dir.exists():
                logger.error(f"❌ No models directory found in {out_dir}")
                return
            
            # Find all models
            models = []
            for item in models_dir.iterdir():
                if item.is_dir():
                    best_model_file = item / 'best_model.pth'
                    if best_model_file.exists():
                        models.append(item)
            
            if not models:
                logger.error(f"❌ No models found in {models_dir}")
                return
            
            # Show interactive selection
            print("🤖 Available models:")
            for i, model in enumerate(sorted(models), 1):
                print(f"  {i:2d}. {model.name}")
            
            try:
                choice = input("\n🎯 Select a model (enter number): ").strip()
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(models):
                    selected_model = sorted(models)[choice_idx]
                    logger.info(f"✅ Selected model: {selected_model.name}")
                    model_name = selected_model.name
                else:
                    logger.error("❌ Invalid selection")
                    return
            except (ValueError, KeyboardInterrupt):
                logger.warn("⚠️  Selection cancelled")
                return
        else:
            model_name = args.model
        
        # Interactive dataset selection if not provided
        if not args.dataset:
            out_dir = Path(args.out_dir)
            datasets_dir = out_dir / "datasets"
            
            if not datasets_dir.exists():
                logger.error(f"❌ No datasets directory found in {out_dir}")
                return
            
            # Find all datasets
            datasets = []
            for item in datasets_dir.iterdir():
                if item.is_dir():
                    metadata_file = item / 'metadata.json'
                    if metadata_file.exists():
                        datasets.append(item)
            
            if not datasets:
                logger.error(f"❌ No datasets found in {datasets_dir}")
                return
            
            # Show interactive selection
            print("📊 Available datasets:")
            for i, dataset in enumerate(sorted(datasets), 1):
                print(f"  {i:2d}. {dataset.name}")
            
            try:
                choice = input("\n🎯 Select a dataset (enter number): ").strip()
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(datasets):
                    selected_dataset = sorted(datasets)[choice_idx]
                    logger.info(f"✅ Selected dataset: {selected_dataset.name}")
                    dataset_path = str(selected_dataset)
                else:
                    logger.error("❌ Invalid selection")
                    return
            except (ValueError, KeyboardInterrupt):
                logger.warn("⚠️  Selection cancelled")
                return
        else:
            dataset_path = str(Path(args.out_dir) / "datasets" / args.dataset)
        
        # Create trainer and evaluate
        try:
            trainer = VisionControlTrainer(
                dataset_dir=dataset_path,
                model_name=model_name,
                out_dir=args.out_dir
            )
            
            trainer.evaluate()
            
        except Exception as e:
            logger.error(f"❌ Evaluation failed: {e}")
    
    elif args.command == 'inference':
        # Interactive model selection if not provided
        if not args.model:
            out_dir = Path(args.out_dir)
            models_dir = out_dir / "models"
            
            if not models_dir.exists():
                logger.error(f"❌ No models directory found in {out_dir}")
                return
            
            # Find all models
            models = []
            for item in models_dir.iterdir():
                if item.is_dir():
                    best_model_file = item / 'best_model.pth'
                    if best_model_file.exists():
                        models.append(item)
            
            if not models:
                logger.error(f"❌ No models found in {models_dir}")
                return
            
            # Show interactive selection
            print("🤖 Available models:")
            for i, model in enumerate(sorted(models), 1):
                print(f"  {i:2d}. {model.name}")
            
            try:
                choice = input("\n🎯 Select a model (enter number): ").strip()
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(models):
                    selected_model = sorted(models)[choice_idx]
                    logger.info(f"✅ Selected model: {selected_model.name}")
                    model_name = selected_model.name
                else:
                    logger.error("❌ Invalid selection")
                    return
            except (ValueError, KeyboardInterrupt):
                logger.warn("⚠️  Selection cancelled")
                return
        else:
            model_name = args.model
        
        # Create and start inferencer
        try:
            inferencer = create_inferencer_from_model_name(
                model_name=model_name,
                out_dir=args.out_dir,
                device=args.device,
                max_linear_speed=args.max_linear,
                max_angular_speed=args.max_angular
            )
            
            print(f"\n🚀 Starting vision control inference...")
            print(f"📦 Model: {model_name}")
            print(f"📊 Rate: {args.rate} Hz")
            print(f"🚗 Speed limits - Linear: {args.max_linear}, Angular: {args.max_angular}")
            print(f"🔧 Device: {args.device}")
            print(f"\nPress Ctrl+C to stop...\n")
            
            inferencer.start_inference(publish_rate=args.rate)
            
        except Exception as e:
            logger.error(f"❌ Inference failed: {e}")
    
    elif args.command == 'test':
        # Interactive model selection if not provided
        if not args.model:
            out_dir = Path(args.out_dir)
            models_dir = out_dir / "models"
            
            if not models_dir.exists():
                logger.error(f"❌ No models directory found in {out_dir}")
                return
            
            # Find all models
            models = []
            for item in models_dir.iterdir():
                if item.is_dir():
                    best_model_file = item / 'best_model.pth'
                    if best_model_file.exists():
                        models.append(item)
            
            if not models:
                logger.error(f"❌ No models found in {models_dir}")
                return
            
            # Show interactive selection
            print("🤖 Available models:")
            for i, model in enumerate(sorted(models), 1):
                print(f"  {i:2d}. {model.name}")
            
            try:
                choice = input("\n🎯 Select a model (enter number): ").strip()
                choice_idx = int(choice) - 1
                if 0 <= choice_idx < len(models):
                    selected_model = sorted(models)[choice_idx]
                    logger.info(f"✅ Selected model: {selected_model.name}")
                    model_name = selected_model.name
                else:
                    logger.error("❌ Invalid selection")
                    return
            except (ValueError, KeyboardInterrupt):
                logger.warn("⚠️  Selection cancelled")
                return
        else:
            model_name = args.model
        
        # Test prediction on single image
        try:
            inferencer = create_inferencer_from_model_name(
                model_name=model_name,
                out_dir=args.out_dir
            )
            
            result = inferencer.predict_single_image(args.image)
            
            if result:
                print(f"\n🔮 Test Prediction Results:")
                print(f"  Linear velocity (x): {result['linear_x']:.4f}")
                print(f"  Angular velocity (z): {result['angular_z']:.4f}")
                print(f"  Inference time: {result['inference_time']*1000:.2f}ms")
            else:
                logger.error("❌ Prediction failed")
                
        except Exception as e:
            logger.error(f"❌ Test prediction failed: {e}")