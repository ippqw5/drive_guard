import math
import carla

from .ros_carla_bridge import *
from .actor import Actor

class Sensor(Actor):
    """
    carla-like DriveGuard Sensor class.
    """
    def __init__(self, name, node: DriveGuardNode):
        self.name = name
        self.node = node

    def get_acceleration(self) -> carla.Vector3D:
        raise NotImplementedError("Sensors do not have acceleration.")

    def get_angular_velocity(self) -> carla.Vector3D:
        raise NotImplementedError("Sensors do not have angular velocity.")

    def get_velocity(self) -> carla.Vector3D:
        raise NotImplementedError("Sensors do not have velocity.")

    def get_transform(self) -> carla.Transform:
        transform = self.node.get_transform('map', self.name)
        if transform is None:
            return None
        return ros_transform_to_carla_transform(transform)

    def get_location(self) -> carla.Location:
        return self.get_transform().location
    
    def get_world(self):
        return self.node.world

    def set_target_angular_velocity(self, angular_velocity: carla.Vector3D):
        raise NotImplementedError("Sensors do not have target angular velocity.")
    
    def set_target_velocity(self, velocity: carla.Vector3D):
        raise NotImplementedError("Sensors do not have target velocity.")
