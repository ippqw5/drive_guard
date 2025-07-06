from PIL import Image
import torchvision.transforms as transforms
import cv2
import numpy as np
from cv_bridge import CvBridge

def resize_image(img_msg, size=(224, 224)):
    """
    Resize ROS Image message to the given size
    
    Args:
        img_msg: ROS Image message
        size: Target size (width, height)
        
    Returns:
        Resized numpy array
    """
    cv_bridge = CvBridge()
    cv_image = cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='rgb8')
    resized_image = cv2.resize(cv_image, size)
    return resized_image

def normalize_image(image):
    """
    Normalize image for neural network input
    
    Args:
        image: Input image as numpy array
        
    Returns:
        Normalized image
    """
    # Normalize to [0, 1]
    normalized = image.astype(np.float32) / 255.0
    
    # Standardize using ImageNet mean and std
    mean = np.array([0.485, 0.456, 0.406])
    std = np.array([0.229, 0.224, 0.225])
    
    # Apply normalization to each channel
    for i in range(3):
        normalized[:, :, i] = (normalized[:, :, i] - mean[i]) / std[i]
    
    return normalized

def preprocess_image(img_msg, size=(224, 224)):
    """
    Complete preprocessing pipeline for ROS Image message
    
    Args:
        img_msg: ROS Image message
        size: Target size (width, height)
        
    Returns:
        Preprocessed numpy array ready for model input
    """
    # Resize
    resized = resize_image(img_msg, size)
    
    # Normalize
    normalized = normalize_image(resized)
    
    # Transpose from HWC to CHW format for PyTorch
    transposed = normalized.transpose((2, 0, 1))
    
    return transposed

def augment_image(image):
    """
    Apply data augmentation to an image
    
    Args:
        image: Input image as numpy array (H, W, C)
        
    Returns:
        Augmented image
    """
    # Random brightness adjustment
    brightness_factor = np.random.uniform(0.8, 1.2)
    image = cv2.convertScaleAbs(image, alpha=brightness_factor, beta=0)
    
    # Random horizontal flip with 50% probability
    if np.random.random() < 0.5:
        image = cv2.flip(image, 1)
    
    # Random slight rotation
    angle = np.random.uniform(-10, 10)
    h, w = image.shape[:2]
    M = cv2.getRotationMatrix2D((w/2, h/2), angle, 1.0)
    image = cv2.warpAffine(image, M, (w, h), borderMode=cv2.BORDER_REFLECT)
    
    return image