from enum import Enum
from abc import ABC, abstractmethod
from carla import Vector3D, Rotation, Location, Transform

from ..driveguard_node import DriveGuardNode
from .driveguard_actor import Actor
from .driveguard_vehicle import Vehicle
from .driveguard_sensor import Sensor


EGO_VEHICLE = int(0)
EGO_IMU = int(1)
EGO_CAMERA = int(2)
EGO_LIDAR = int(3)


class World():
    _instance = None
    _initialized = False

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(World, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if World._initialized:
            return

        self.id = 0
        self.debug = None
        
        self.node = DriveGuardNode()
        self.node.world = self
        self.ego_vehicle = Vehicle(name="vehicle", node=self.node)
        self.ego_imu = Sensor(name="imu", node=self.node)
        self.ego_camera = Sensor(name="camera", node=self.node)
        self.ego_lidar = Sensor(name="lidar", node=self.node)
        World._initialized = True

    def __str__(self):
        return f"World(id={self.id})"

    def get_actor(self, actor_id: int) -> Actor | None:
        """
        Looks up for an actor by ID and returns None if not found. 

        Args:
            actor_id (int)

        Returns:
            Actor | None: The actor instance or None if not found.
        """
        if actor_id == EGO_VEHICLE:
            return self.ego_vehicle
        else:
            sensor = self.get_sensor(actor_id)
            if sensor:
                return sensor
        return None

    def get_sensor(self, sensor_id: int) -> Sensor | None:
        """
        Get the sensor instance by type.

        Args:
            sensor_id (int): The ID of the sensor to get.

        Returns:
            Sensor | None: The sensor instance or None if not found.
        """
        sensor_map = {
            EGO_IMU: self.ego_imu,
            EGO_CAMERA: self.ego_camera,
            EGO_LIDAR: self.ego_lidar
        }
        return sensor_map.get(sensor_id, None)

    def shutdown(self):
        """
        Shutdown the world and clean up resources.
        """
        if self.node:
            self.node.shutdown()
            self.node = None
        self.ego_vehicle = None
        self.ego_imu = None
        self.ego_camera = None
        self.ego_lidar = None

class Client():
    def __init__(self):
        self.world = World()

    def get_world(self) -> World:
        """
        Get the singleton instance of the World.

        Returns:
            World: The singleton instance of the World.
        """
        return self.world