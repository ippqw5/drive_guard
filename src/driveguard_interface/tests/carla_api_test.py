import sys
import os
import time
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))
  
import driveguard.carla as carla

if __name__ == "__main__":

    client = carla.Client()
    
    world = client.get_world()

    ego_vehicle_id = carla.EGO_VEHICLE
    ego_vehicle = world.get_actor(ego_vehicle_id)

    print(carla.Vector3D(1.0, 2.0, 3.0))

    try:
        while True:
            time.sleep(0.5)
            print(f"Ego Vehicle Location: {ego_vehicle.get_location()}")
            print(f"Ego Vehicle Velocity: {ego_vehicle.get_velocity()}")
            print(f"Ego Vehicle Acceleration: {ego_vehicle.get_acceleration()}")
            print(f"Ego Vehicle Angular Velocity: {ego_vehicle.get_angular_velocity()}")
            print(f"Ego Vehicle Transform: {ego_vehicle.get_transform()}")
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received")
    finally:
        world.shutdown()