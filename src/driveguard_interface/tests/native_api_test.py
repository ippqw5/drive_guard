import sys
import os
import time
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
  
from driveguard_node import DriveGuardNode


def image_test(node : DriveGuardNode):
    image_tensor = node.get_image_tensor()

    print(f"image tensor shape: {image_tensor.shape}")
    print(f"image tensor dtype: {image_tensor.dtype}")

    # Convert the tensor to a numpy array and transpose the dimensions
    image_np = image_tensor.squeeze().permute(1, 2, 0).cpu().numpy()

    plt.imshow(image_np)
    plt.axis('off')
    plt.show()

def cmd_vel_test(node : DriveGuardNode):
    node.set_twist(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    node.set_twist_ai(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


if __name__ == "__main__":
    node = DriveGuardNode()
    
    image_test(node)
    cmd_vel_test(node)

    node.shutdown()