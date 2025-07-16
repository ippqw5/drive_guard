import math
from .carla_types import Vector3D, Location, Transform

from .ros_carla_bridge import *
from .actor import Actor

class Vehicle(Actor):
    """
    carla-like DriveGuard Vehicle class.
    """
    def __init__(self, name, node: DriveGuardNode):
        self.name = name
        self.node = node
        
#################################################################
#################################################################
#################################################################
    def get_acceleration(self) -> Vector3D:
        accel = self.node.get_linear_acceleration()
        if accel is None:
            return Vector3D(0, 0, 0)
        return ros_vector3_to_carla_vector(accel)
    
    def get_angular_velocity(self) -> Vector3D:
        angular_velocity = self.node.get_angular_velocity()
        if angular_velocity is None:
            return Vector3D(0, 0, 0)
        return ros_angular_velocity_to_carla_vector(angular_velocity)
    
    def get_velocity(self) -> Vector3D:
        vel = self.node.get_linear_velocity()
        if vel is None:
            return Vector3D(0, 0, 0)
        return ros_vector3_to_carla_vector(vel)

    def get_transform(self) -> Transform:
        pose = self.node.get_pose()
        if pose is None:
            return Transform()
        return ros_pose_to_carla_transform(pose)

    def get_location(self) -> Location:
        transform = self.get_transform()
        if transform is None:
            return Location()
        return transform.location

    def get_world(self):
        return self.node.world

    def set_target_velocity(self, velocity: Vector3D):
        velocity_ros = carla_vector_to_ros_vector3(velocity)

        twist_ros = Twist()
        twist_ros.linear.x = velocity_ros.x
        twist_ros.linear.y = velocity_ros.y
        twist_ros.linear.z = velocity_ros.z

        self.node.set_twist(twist_ros)

    def set_target_angular_velocity(self, angular_velocity: Vector3D):
        angular_velocity_ros = carla_vector_to_ros_vector3(angular_velocity)

        twist_ros = Twist()
        twist_ros.angular = angular_velocity_ros

        self.node.set_twist(twist_ros)
#################################################################
#################################################################
#################################################################