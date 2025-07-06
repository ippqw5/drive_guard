import torch
import torch.nn as nn
import torchvision.models as models

class DriveGuardResNet18(nn.Module):
    def __init__(self, num_outputs=2, pretrained=False):
        """
        Initialize the DriveGuard ResNet18 model
        
        Args:
            num_outputs (int): Number of output values (2 for linear_x and angular_z)
            pretrained (bool): Whether to use pretrained weights
        """
        super(DriveGuardResNet18, self).__init__()
        
        # Load pre-trained ResNet18
        if pretrained:
            self.resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        else:
            self.resnet = models.resnet18()
        
        # Replace the final fully connected layer
        num_features = self.resnet.fc.in_features
        self.resnet.fc = nn.Sequential(
            nn.Linear(num_features, 256),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256, num_outputs)
        )
    
    def forward(self, x):
        return self.resnet(x)

# Additional model architectures can be added here as needed
class DriveGuardMobileNetV2(nn.Module):
    def __init__(self, num_outputs=2, pretrained=False):
        """
        Initialize the DriveGuard MobileNetV2 model - a more lightweight alternative
        
        Args:
            num_outputs (int): Number of output values (2 for linear_x and angular_z)
            pretrained (bool): Whether to use pretrained weights
        """
        super(DriveGuardMobileNetV2, self).__init__()
        
        # Load pre-trained MobileNetV2
        if pretrained:
            self.mobilenet = models.mobilenet_v2(weights=models.MobileNet_V2_Weights.DEFAULT)
        else:
            self.mobilenet = models.mobilenet_v2()
        
        # Replace the final classifier
        self.mobilenet.classifier = nn.Sequential(
            nn.Dropout(0.2),
            nn.Linear(self.mobilenet.last_channel, num_outputs)
        )
    
    def forward(self, x):
        return self.mobilenet(x)