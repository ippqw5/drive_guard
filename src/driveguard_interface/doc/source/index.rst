.. DriveGuard documentation master file, created by
   sphinx-quickstart on Thu Jun  5 03:18:00 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

DriveGuard API documentation
=======================================================

Welcome to the DriveGuard API documentation!

DriveGuard provides a set of easy-to-use Python API for controlling and monitoring our real car.

In the class **drive_guard**, you can find many get/set methods which hide the complexity of ROS2 and the underlying hardware.

=======================================================

Here is a sample code snippet to get you started:

.. code-block:: python

   # Import the driveguard_interface module
   from driveguard_interface import *

   # Create an instance of the DriveGuard class to control the car
   drive_guard = DriveGuard()

   try:
      # Get speed and twist in target frame
      while True:
         time.sleep(0.5) 
         vel = drive_guard.get_vel()
         if vel:
               print(f"Linear: {vel.linear.x}, {vel.linear.y}, {vel.linear.z} | "
                     f"Angular: {vel.angular.x}, {vel.angular.y}, {vel.angular.z}")
         else:
               print("No velocity data available")

   except KeyboardInterrupt:
      print("\nKeyboard interrupt received")
   finally:
      # Stop the car and shutdown the DriveGuard instance
      drive_guard.stop_robot()
      drive_guard.shutdown()

=======================================================

.. toctree::
   :maxdepth: 2
   :name: mastertoc
   
   🏠 Home <self>
   📚 API Reference <drive_guard>
   ⚙️ API ROS2 Node Implementation <drive_guard_node>
