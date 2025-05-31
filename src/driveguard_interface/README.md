# DriveGuard Interface

DriveGuard provides a set of easy-to-use Python API for controlling and monitoring real cars/robots.

## Quick Start

```python

# Import the driveguard_interface module
from driveguard_interface import *

drive_guard = DriveGuard()

# Get IMU data
imu_data = drive_guard.get_imu_data()
if imu_data:
    # print(f"Type: {type(imu_data)}")
    print("IMU Data:", imu_data)
else:
    print("No IMU data received yet")

try:
    # Get speed and twist in target frame
    while True:
        time.sleep(0.5) 
        
        pose = drive_guard.get_pose()
        vel = drive_guard.get_vel()
        
        if pose:
            print(f"Pose: {pose.vector.x}, {pose.vector.y}, {pose.vector.z}")
        else:
            print("No pose data available")
        if vel:
            print(f"Linear: {vel.linear.x}, {vel.linear.y}, {vel.linear.z} | "
                    f"Angular: {vel.angular.x}, {vel.angular.y}, {vel.angular.z}")
        else:
            print("No velocity data available")

except KeyboardInterrupt:
    print("\nKeyboard interrupt received")
finally:
    drive_guard.stop_robot()
    drive_guard.shutdown()
```

## Features

- Easy-to-use API that hides ROS2 complexity
- Real-time sensor data access (IMU, Camera, Odometry)
- Robot control capabilities
- Transform handling between coordinate frames

## Documentation

For detailed documentation, visit [our docs](http://219.228.60.63:218/).