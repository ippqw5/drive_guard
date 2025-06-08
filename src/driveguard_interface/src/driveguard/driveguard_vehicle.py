import math
import carla

from .driveguard_carla_bridge import *
from .driveguard_actor import Actor

class Vehicle(Actor):
    """
    carla-like DriveGuard Vehicle class.
    """
    def __init__(self, name, node: DriveGuardNode):
        self.name = name
        self.node = node

    def get_acceleration(self) -> carla.Vector3D:
        """
        Get the acceleration of the vehicle.
        
        Returns:
            carla.Vector3D - m/s^2
        """
        accel = self.node.get_linear_acceleration()
        if accel is None:
            return None
        return ros_vector3_to_carla_vector(accel)
    
    def get_angular_velocity(self) -> carla.Vector3D:
        """
        Get the angular velocity of the vehicle.
        
        Returns:
            carla.Vector3D - deg/s
        """
        angular_velocity = self.node.get_angular_velocity()
        if angular_velocity is None:
            return None
        return ros_angular_velocity_to_carla_vector(angular_velocity)
    
    def get_velocity(self) -> carla.Vector3D:
        """ 
        Get the velocity of the vehicle.

        Returns:
            carla.Vector3D - m/s.
        """
        vel = self.node.get_linear_velocity()
        if vel is None:
            return None
        return ros_vector3_to_carla_vector(vel)

    def get_transform(self) -> carla.Transform:
        """
        Get the transform of the vehicle.
        
        Returns:
            carla.Transform - The transform of the vehicle.
        """
        pose = self.node.get_pose()
        if pose is None:
            return None
        return ros_pose_to_carla_transform(pose)

    def get_location(self) -> carla.Location:
        """
        Get the location of the vehicle.
        
        Returns:
            carla.Location - The location of the vehicle.
        """
        transform = self.get_transform()
        if transform is None:
            return None
        return transform.location

    def get_world(self):
        """
        Get the world instance associated with the vehicle.
        
        Returns:
            World - The world instance.
        """
        return self.node.world