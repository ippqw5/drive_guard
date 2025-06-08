import math
import carla

from .driveguard_carla_bridge import *
from .driveguard_actor import Actor

class Sensor(Actor):
    """
    carla-like DriveGuard Sensor class.
    """
    def __init__(self, name, node: DriveGuardNode):
        self.name = name
        self.node = node

    def get_acceleration(self) -> carla.Vector3D:
        """
        Not implemented for Sensor.
        """
        raise NotImplementedError("Sensors do not have acceleration.")

    def get_angular_velocity(self) -> carla.Vector3D:
        """
        Not implemented for Sensor.
        """
        raise NotImplementedError("Sensors do not have angular velocity.")

    def get_velocity(self) -> carla.Vector3D:
        """
        Not implemented for Sensor.
        """
        raise NotImplementedError("Sensors do not have velocity.")

    def get_transform(self) -> carla.Transform:
        """
        Get the transform of the sensor.

        Returns:
            carla.Transform - The transform of the sensor.
        """
        transform = self.node.get_transform('map', self.name)
        if transform is None:
            return None
        return ros_transform_to_carla_transform(transform)

    def get_location(self) -> carla.Location:
        """
        Get the location of the sensor.

        Returns:
            carla.Location - The location of the sensor.
        """
        return self.get_transform().location
    
    def get_world(self):
        """
        Get the world instance associated with the vehicle.
        
        Returns:
            World - The world instance.
        """
        return self.node.world
