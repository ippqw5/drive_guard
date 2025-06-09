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
