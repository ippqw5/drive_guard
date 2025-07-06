from ..driveguard_node import DriveGuardNode

import math
import numpy
import time

import carla

import rclpy
from geometry_msgs.msg import Vector3, Quaternion, Transform, Pose, Point, Twist, Accel
from transforms3d.euler import euler2mat, quat2euler, euler2quat
from transforms3d.quaternions import quat2mat, mat2quat
from sensor_msgs.msg import Imu, Image

#################################################################
#################################################################
#################################################################
def ros_point_to_carla_location(ros_point : Point) -> carla.Location:
    """
    Convert a ROS Point to a CARLA Location.
    
    Args:
        ros_point (Point): The ROS Point to convert.
    
    Returns:
        carla.Location: The converted CARLA Location.
    """
    return carla.Location(ros_point.x, -ros_point.y, ros_point.z)


def RPY_to_carla_rotation(roll, pitch, yaw) -> carla.Rotation:
    """
    Convert roll, pitch, yaw angles to a CARLA Rotation.
    
    Args:
        roll (float): Roll angle in radians.
        pitch (float): Pitch angle in radians.
        yaw (float): Yaw angle in radians.
        
    Returns:
        carla.Rotation: The CARLA Rotation object.
    """
    return carla.Rotation(roll=math.degrees(roll),
                          pitch=-math.degrees(pitch),
                          yaw=-math.degrees(yaw))


def ros_quaternion_to_carla_rotation(ros_quaternion : Quaternion) -> carla.Rotation:
    """
    Convert a ROS Quaternion to a CARLA Rotation.
    
    Args:
        ros_quaternion (Quaternion): The ROS Quaternion to convert.
    
    Returns:
        carla.Rotation: The converted CARLA Rotation.
    """
    roll, pitch, yaw = quat2euler([ros_quaternion.w,
                                   ros_quaternion.x,
                                   ros_quaternion.y,
                                   ros_quaternion.z])
    return RPY_to_carla_rotation(roll, pitch, yaw)

def ros_transform_to_carla_transform(ros_transform : Transform) -> carla.Transform:
    """
    Convert a ROS Transform to a CARLA Transform.
    
    Args:
        ros_transform (Transform): The ROS Transform to convert.
        
    Returns:
        carla.Transform: The converted CARLA Transform.
    """
    return carla.Transform(
        ros_point_to_carla_location(ros_transform.translation),
        ros_quaternion_to_carla_rotation(ros_transform.rotation))

def ros_pose_to_carla_transform(ros_pose : Pose) -> carla.Transform:
    """
    Convert a ROS Pose to a CARLA Transform.
    
    Args:
        ros_pose (Pose): The ROS Pose to convert.
        
    Returns:
        carla.Transform: The converted CARLA Transform.
    """
    return carla.Transform(
        ros_point_to_carla_location(ros_pose.position),
        ros_quaternion_to_carla_rotation(ros_pose.orientation))


def ros_vector3_to_carla_vector(ros_vector : Vector3) -> carla.Vector3D:
    """
    Convert a ROS Vector3 to a CARLA Vector3D.
    
    Args:
        ros_vector (Vector3): The ROS Vector3 to convert.
        
    Returns:
        carla.Vector3D: The converted CARLA Vector3D.
    """
    return carla.Vector3D(ros_vector.x, -ros_vector.y, ros_vector.z)


def ros_angular_velocity_to_carla_vector(ros_angular_velocity : Vector3) -> carla.Vector3D:
    """
    Convert a ROS AngularVelocity to a CARLA Vector3D.
    
    Args:
        ros_angular_velocity (Vector3): The ROS AngularVelocity to convert.
        
    Returns:
        carla.Vector3D: The converted CARLA Vector3D.
    """
    return carla.Vector3D(
        ros_angular_velocity.x * 180 / math.pi,
        -ros_angular_velocity.y * 180 / math.pi,
        ros_angular_velocity.z * 180 / math.pi
    )
    
#################################################################
#################################################################
#################################################################

def carla_vector_to_ros_vector3(carla_vector: carla.Vector3D) -> Vector3:
    """
    Convert a CARLA Vector3D to a ROS Vector3.
    
    Args:
        carla_vector (carla.Vector3D): The CARLA Vector3D to convert.
        
    Returns:
        Vector3: The converted ROS Vector3.
    """
    return Vector3(x=carla_vector.x, y=-carla_vector.y, z=carla_vector.z)