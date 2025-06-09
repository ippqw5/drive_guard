from enum import Enum
from abc import ABC, abstractmethod
from carla import Vector3D, Rotation, Location, Transform

from ..driveguard_node import DriveGuardNode
from .driveguard_actor import Actor
from .driveguard_vehicle import Vehicle
from .driveguard_sensor import Sensor


EGO_VEHICLE = "vehicle"
EGO_IMU = "imu"
EGO_CAMERA = "camera"
EGO_LIDAR = "lidar"


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

        self.node = DriveGuardNode()
        self.node.world = self
        self.ego_vehicle = Vehicle(name="vehicle", node=self.node)
        self.ego_imu = Sensor(name="imu", node=self.node)
        self.ego_camera = Sensor(name="camera", node=self.node)
        self.ego_lidar = Sensor(name="lidar", node=self.node)
        World._initialized = True

    def get_actor(self, actor: str) -> Actor | None:
        """
        Get the actor instance by type.

        Args:
            actor (str): The type of actor to get.

        Returns:
            Vehicle | Sensor | None: The actor instance or None if not found.
        """
        if actor == EGO_VEHICLE:
            return self.ego_vehicle
        else:
            sensor = self.get_sensor(actor)
            if sensor:
                return sensor
        return None

    def get_sensor(self, sensor: str) -> Sensor | None:
        """
        Get the sensor instance by type.

        Args:
            sensor (str): The type of sensor to get.

        Returns:
            Sensor | None: The sensor instance or None if not found.
        """
        sensor_map = {
            EGO_IMU: self.ego_imu,
            EGO_CAMERA: self.ego_camera,
            EGO_LIDAR: self.ego_lidar
        }
        return sensor_map.get(sensor, None)

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