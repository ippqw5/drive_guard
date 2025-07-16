import math
from .carla_types import Vector3D, Location, Transform

from .ros_carla_bridge import *
from .actor import Actor

class Sensor(Actor):
    """
    carla-like DriveGuard Sensor class.
    """
    def __init__(self, name, node: DriveGuardNode):
        self.name = name
        self.node = node

    def get_acceleration(self) -> Vector3D:
        raise NotImplementedError("Sensors do not have acceleration.")

    def get_angular_velocity(self) -> Vector3D:
        raise NotImplementedError("Sensors do not have angular velocity.")

    def get_velocity(self) -> Vector3D:
        raise NotImplementedError("Sensors do not have velocity.")

    def get_transform(self) -> Transform:
        transform = self.node.get_transform('map', self.name)
        if transform is None:
            return Transform()
        return ros_transform_to_carla_transform(transform.transform)

    def get_location(self) -> Location:
        return self.get_transform().location
    
    def get_world(self):
        return self.node.world

    def set_target_angular_velocity(self, angular_velocity: Vector3D):
        raise NotImplementedError("Sensors do not have target angular velocity.")
    
    def set_target_velocity(self, velocity: Vector3D):
        raise NotImplementedError("Sensors do not have target velocity.")
