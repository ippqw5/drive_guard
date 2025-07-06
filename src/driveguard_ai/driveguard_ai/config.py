# Directory structure
DATASETS_DIR = "./out/datasets"
MODELS_DIR = "./out/models"

# Data Collection Parameters
DATA_COLLECTION = {
    "topics_to_record": [
        "/camera_sensor/image_raw",
        "/cmd_vel"
    ],
    "recording_duration": 30,  # seconds, 0 for unlimited
    "max_bag_size": 1000,  # MB
}

# Model Training Parameters
TRAINING = {
    "batch_size": 32,
    "learning_rate": 0.001,
    "num_epochs": 20,
    "val_split": 0.2,
    "image_size": [224, 224],
}

# Inference Parameters
INFERENCE = {
    "image_topic": "/camera_sensor/image_raw",
    "cmd_topic": "/cmd_vel",
    "device": "auto",  # "auto", "cuda", or "cpu"
}
