.. DriveGuard documentation master file, created by
   sphinx-quickstart on Thu Jun  5 03:18:00 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

DriveGuard API documentation
=======================================================

Welcome to the DriveGuard API documentation!

DriveGuard API provides two comprehensive Python API sets for interacting with our real vehicle:

**Carla-Compatible API**
   Built on top of the `Carla Python API <https://carla.readthedocs.io/en/0.9.14/python_api/>`_ and implemented using ROS2. If your application is based on Carla simulation, you can seamlessly migrate it to our real vehicle with minimal code changes.

**Native API**
   Our custom-designed API with proprietary functions, inputs, and outputs. This API is specifically tailored for optimal performance and direct control of our vehicle system.

=======================================================

Here is a sample code snippet to get you started:

.. code-block:: python

   import driveguard_interface.driveguard_carla as carla

   client = carla.Client()
   
   world = client.get_world()

   ego_vehicle_id = carla.EGO_VEHICLE
   ego_vehicle = world.get_actor(ego_vehicle_id)

   print(f"Ego Vehicle Location: {ego_vehicle.get_location()}")
   print(f"Ego Vehicle Velocity: {ego_vehicle.get_velocity()}")
   print(f"Ego Vehicle Acceleration: {ego_vehicle.get_acceleration()}")
   print(f"Ego Vehicle Angular Velocity: {ego_vehicle.get_angular_velocity()}")
   print(f"Ego Vehicle Transform: {ego_vehicle.get_transform()}")


=======================================================

.. toctree::
   :maxdepth: 1
   :caption: Contents

   Welcome <self>
   Carla-Compatible API Reference <carla_api/index>
   Native API Reference <native_api/index>
