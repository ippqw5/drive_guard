import carla
from abc import ABC, abstractmethod

class Actor(ABC):
    """
    Abstract base class for DriveGuard actors.
    """
    
    @abstractmethod
    def get_transform(self) -> carla.Transform:
        """
        Get the transform of the actor.
        
        Returns:
            carla.Transform: The transform of the actor.
        """
        pass

    @abstractmethod
    def get_location(self) -> carla.Location:
        """
        Get the location of the actor.
        
        Returns:
            carla.Location: The location of the actor.
        """
        pass
    
    @abstractmethod
    def get_velocity(self) -> carla.Vector3D:
        """
        Get the velocity of the actor.
        
        Returns:
            carla.Vector3D: The velocity of the actor.
        """
        pass

    @abstractmethod
    def get_angular_velocity(self) -> carla.Vector3D:
        """
        Get the angular velocity of the actor.

        Returns:
            carla.Vector3D: The angular velocity of the actor.
        """
        pass
        """
        Get the angular velocity of the actor.
        
        Returns:
            carla.Vector3D: The angular velocity of the actor.
        """
        pass
    
    @abstractmethod
    def get_acceleration(self) -> carla.Vector3D:
        """
        Get the acceleration of the actor.
        
        Returns:
            carla.Vector3D: The acceleration of the actor.
        """
        pass
    
    @abstractmethod
    def get_world(self):
        """
        Get the world in which the actor exists.
        
        Returns:
            driveguard.World: The world of the actor.
        """
        pass

    def set_enable_gravity(self, enabled: bool):
        """
        Enables or disables gravity for the actor. Default is True.
        
        Args:
            enabled (bool): Whether to enable gravity.
        """
        return True
    
    def set_location(self, location: carla.Location):
        """
        Teleports the actor to a given location.
        
        Args:
            location (carla.Location): The new location of the actor.
        """
        raise NotImplementedError("This method can not be implemented on DriveGuard.")
    
    def set_simulate_physics(self, enabled: bool = True):
        """
        Enables or disables the simulation of physics on this actor.
        
        Args:
            enabled (bool): Whether to enable physics simulation.
        """
        return True
    
    @abstractmethod
    def set_target_angular_velocity(self, angular_velocity: carla.Vector3D):
        """
        Sets the actor's angular velocity vector. This is applied before the physics step so the resulting angular velocity will be affected by external forces such as friction.
        
        Args:
            angular_velocity (carla.Vector3D): The target angular velocity in degrees per second.
        """
        pass
    
    @abstractmethod
    def set_target_velocity(self, velocity: carla.Vector3D):
        """
        Sets the actor's velocity vector. This is applied before the physics step so the resulting angular velocity will be affected by external forces such as friction.
        
        Args:
            velocity (carla.Vector3D): The target velocity.
        """
        pass
    
    def set_transform(self, transform: carla.Transform):
        """
        Teleports the actor to a given transform (location and rotation).
        
        Args:
            transform (carla.Transform): The new transform of the actor.
        """
        raise NotImplementedError("This method can not be implemented on DriveGuard.")