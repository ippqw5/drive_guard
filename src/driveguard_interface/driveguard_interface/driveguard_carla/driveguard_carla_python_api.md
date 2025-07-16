## carla.Actor<a name="carla.Actor"></a>
CARLA defines actors as anything that plays a role in the simulation or can be moved around. That includes: pedestrians, vehicles, sensors and traffic signs (considering traffic lights as part of these). Actors are spawned in the simulation by [carla.World](#carla.World) and they need for a [carla.ActorBlueprint](#carla.ActorBlueprint) to be created. These blueprints belong into a library provided by CARLA, find more about them [here](bp_library.md).  

### Instance Variables
- <a name="carla.Actor.attributes"></a>**<font color="#f8805a">attributes</font>** (_dict_)  
A dictionary containing the attributes of the blueprint this actor was based on.  
- <a name="carla.Actor.id"></a>**<font color="#f8805a">id</font>** (_int_)  
Identifier for this actor. Unique during a given episode.  
- <a name="carla.Actor.is_alive"></a>**<font color="#f8805a">is_alive</font>** (_bool_)  
Returns whether this object was destroyed using this actor handle.  
- <a name="carla.Actor.parent"></a>**<font color="#f8805a">parent</font>** (_[carla.Actor](#carla.Actor)_)  
Actors may be attached to a parent actor that they will follow around. This is said actor.  
- <a name="carla.Actor.semantic_tags"></a>**<font color="#f8805a">semantic_tags</font>** (_list(int)_)  
A list of semantic tags provided by the blueprint listing components for this actor. E.g. a traffic light could be tagged with `Pole` and `TrafficLight`. These tags are used by the semantic segmentation sensor. Find more about this and other sensors [here](ref_sensors.md#semantic-segmentation-camera).  
- <a name="carla.Actor.type_id"></a>**<font color="#f8805a">type_id</font>** (_str_)  
The identifier of the blueprint this actor was based on, e.g. `vehicle.ford.mustang`.  

### Methods
- <a name="carla.Actor.add_angular_impulse"></a>**<font color="#7fb800">add_angular_impulse</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**angular_impulse**</font>)  
Applies an angular impulse at the center of mass of the actor. This method should be used for instantaneous torques, usually applied once. Use __<font color="#7fb800">add_torque()</font>__ to apply rotation forces over a period of time.  
    - **Parameters:**
        - `angular_impulse` (_[carla.Vector3D](#carla.Vector3D)<small> – degrees*s</small>_) – Angular impulse vector in global coordinates.  
- <a name="carla.Actor.add_force"></a>**<font color="#7fb800">add_force</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**force**</font>)  
Applies a force at the center of mass of the actor. This method should be used for forces that are applied over a certain period of time. Use __<font color="#7fb800">add_impulse()</font>__ to apply an impulse that only lasts an instant.  
    - **Parameters:**
        - `force` (_[carla.Vector3D](#carla.Vector3D)<small> – N</small>_) – Force vector in global coordinates.  
- <a name="carla.Actor.add_impulse"></a>**<font color="#7fb800">add_impulse</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**impulse**</font>)  
Applies an impulse at the center of mass of the actor. This method should be used for instantaneous forces, usually applied once. Use __<font color="#7fb800">add_force()</font>__ to apply forces over a period of time.  
    - **Parameters:**
        - `impulse` (_[carla.Vector3D](#carla.Vector3D)<small> – N*s</small>_) – Impulse vector in global coordinates.  
- <a name="carla.Actor.add_torque"></a>**<font color="#7fb800">add_torque</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**torque**</font>)  
Applies a torque at the center of mass of the actor. This method should be used for torques that are applied over a certain period of time. Use __<font color="#7fb800">add_angular_impulse()</font>__ to apply a torque that only lasts an instant.  
    - **Parameters:**
        - `torque` (_[carla.Vector3D](#carla.Vector3D)<small> – degrees</small>_) – Torque vector in global coordinates.  
- <a name="carla.Actor.destroy"></a>**<font color="#7fb800">destroy</font>**(<font color="#00a6ed">**self**</font>)  
Tells the simulator to destroy this actor and returns <b>True</b> if it was successful. It has no effect if it was already destroyed.  
    - **Return:** _bool_  
    - **Warning:** <font color="#ED2F2F">_This method blocks the script until the destruction is completed by the simulator.
_</font>  
- <a name="carla.Actor.disable_constant_velocity"></a>**<font color="#7fb800">disable_constant_velocity</font>**(<font color="#00a6ed">**self**</font>)  
Disables any constant velocity previously set for a [carla.Vehicle](#carla.Vehicle) actor.  
- <a name="carla.Actor.enable_constant_velocity"></a>**<font color="#7fb800">enable_constant_velocity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**velocity**</font>)  
Sets a vehicle's velocity vector to a constant value over time. The resulting velocity will be approximately the `velocity` being set, as with __<font color="#7fb800">set_target_velocity()</font>__.  
    - **Parameters:**
        - `velocity` (_[carla.Vector3D](#carla.Vector3D)<small> – m/s</small>_) – Velocity vector in local space.  
    - **Warning:** <font color="#ED2F2F">_Enabling a constant velocity for a vehicle managed by the [Traffic Manager](https://[carla.readthedocs.io](#carla.readthedocs.io)/en/latest/adv_traffic_manager/) may cause conflicts. This method overrides any changes in velocity by the TM.  
_</font>  

##### Getters
- <a name="carla.Actor.get_acceleration"></a>**<font color="#7fb800">get_acceleration</font>**(<font color="#00a6ed">**self**</font>)  
Returns the actor's 3D acceleration vector the client recieved during last tick. The method does not call the simulator.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)<small> – m/s<sup>2</sup></small>_  
- <a name="carla.Actor.get_angular_velocity"></a>**<font color="#7fb800">get_angular_velocity</font>**(<font color="#00a6ed">**self**</font>)  
Returns the actor's angular velocity vector the client recieved during last tick. The method does not call the simulator.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)<small> – deg/s</small>_  
- <a name="carla.Actor.get_location"></a>**<font color="#7fb800">get_location</font>**(<font color="#00a6ed">**self**</font>)  
Returns the actor's location the client recieved during last tick. The method does not call the simulator.  
    - **Return:** _[carla.Location](#carla.Location)<small> – meters</small>_  
    - **Setter:** _[carla.Actor.set_location](#carla.Actor.set_location)_  
- <a name="carla.Actor.get_transform"></a>**<font color="#7fb800">get_transform</font>**(<font color="#00a6ed">**self**</font>)  
Returns the actor's transform (location and rotation) the client recieved during last tick. The method does not call the simulator.  
    - **Return:** _[carla.Transform](#carla.Transform)_  
    - **Setter:** _[carla.Actor.set_transform](#carla.Actor.set_transform)_  
- <a name="carla.Actor.get_velocity"></a>**<font color="#7fb800">get_velocity</font>**(<font color="#00a6ed">**self**</font>)  
Returns the actor's velocity vector the client recieved during last tick. The method does not call the simulator.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)<small> – m/s</small>_  
- <a name="carla.Actor.get_world"></a>**<font color="#7fb800">get_world</font>**(<font color="#00a6ed">**self**</font>)  
Returns the world this actor belongs to.  
    - **Return:** _[carla.World](#carla.World)_  

##### Setters
- <a name="carla.Actor.set_location"></a>**<font color="#7fb800">set_location</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>)  
Teleports the actor to a given location.  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)<small> – meters</small>_)  
    - **Getter:** _[carla.Actor.get_location](#carla.Actor.get_location)_  
- <a name="carla.Actor.set_simulate_physics"></a>**<font color="#7fb800">set_simulate_physics</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**=True</font>)  
Enables or disables the simulation of physics on this actor.  
    - **Parameters:**
        - `enabled` (_bool_)  
- <a name="carla.Actor.set_target_angular_velocity"></a>**<font color="#7fb800">set_target_angular_velocity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**angular_velocity**</font>)  
Sets the actor's angular velocity vector. The modification will be effective two frames after the setting. Also, this is applied before the physics step so the resulting angular velocity will be affected by external forces at this frame such as friction.  
    - **Parameters:**
        - `angular_velocity` (_[carla.Vector3D](#carla.Vector3D)<small> – deg/s</small>_)  
    - **Note:** <font color="#8E8E8E">_The update will not be effective until two frames after it is set.  
_</font>  
- <a name="carla.Actor.set_target_velocity"></a>**<font color="#7fb800">set_target_velocity</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**velocity**</font>)  
Sets the actor's velocity vector. The modification will be effective two frames after the setting. Also, this is applied before the physics step so the resulting velocity will be affected by external forces at this frame such as friction.  
    - **Parameters:**
        - `velocity` (_[carla.Vector3D](#carla.Vector3D)_)  
    - **Note:** <font color="#8E8E8E">_The update will not be effective until two frames after it is set.  
_</font>  
- <a name="carla.Actor.set_transform"></a>**<font color="#7fb800">set_transform</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**transform**</font>)  
Teleports the actor to a given transform (location and rotation).  
    - **Parameters:**
        - `transform` (_[carla.Transform](#carla.Transform)_)  
    - **Getter:** _[carla.Actor.get_transform](#carla.Actor.get_transform)_  

##### Dunder methods
- <a name="carla.Actor.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Vehicle<a name="carla.Vehicle"></a>
<div class="Inherited"><small><b>Inherited from _[carla.Actor](#carla.Actor)_</b></small></div>One of the most important group of actors in CARLA. These include any type of vehicle from cars to trucks, motorbikes, vans, bycicles and also official vehicles such as police cars. A wide set of these actors is provided in [carla.BlueprintLibrary](#carla.BlueprintLibrary) to facilitate differente requirements. Vehicles can be either manually controlled or set to an autopilot mode that will be conducted client-side by the <b>traffic manager</b>.  

### Instance Variables
- <a name="carla.Vehicle.bounding_box"></a>**<font color="#f8805a">bounding_box</font>** (_[carla.BoundingBox](#carla.BoundingBox)_)  
Bounding box containing the geometry of the vehicle. Its location and rotation are relative to the vehicle it is attached to.  

### Methods
- <a name="carla.Vehicle.apply_control"></a>**<font color="#7fb800">apply_control</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**control**</font>)  
Applies a control object on the next tick, containing driving parameters such as throttle, steering or gear shifting.  
    - **Parameters:**
        - `control` (_[carla.VehicleControl](#carla.VehicleControl)_)  
- <a name="carla.Vehicle.apply_physics_control"></a>**<font color="#7fb800">apply_physics_control</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**physics_control**</font>)  
Applies a physics control object in the next tick containing the parameters that define the vehicle as a corporeal body. E.g.: moment of inertia, mass, drag coefficient and many more.  
    - **Parameters:**
        - `physics_control` (_[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl)_)  
- <a name="carla.Vehicle.enable_carsim"></a>**<font color="#7fb800">enable_carsim</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**simfile_path**</font>)  
Enables the CarSim physics solver for this particular vehicle. In order for this function to work, there needs to be a valid license manager running on the server side. The control inputs are redirected to CarSim which will provide the position and orientation of the vehicle for every frame.  
    - **Parameters:**
        - `simfile_path` (_str_) – Path to the `.simfile` file with the parameters of the simulation.  
- <a name="carla.Vehicle.is_at_traffic_light"></a>**<font color="#7fb800">is_at_traffic_light</font>**(<font color="#00a6ed">**self**</font>)  
Vehicles will be affected by a traffic light when the light is red and the vehicle is inside its bounding box. The client returns whether a traffic light is affecting this vehicle according to last tick (it does not call the simulator).  
    - **Return:** _bool_  
- <a name="carla.Vehicle.use_carsim_road"></a>**<font color="#7fb800">use_carsim_road</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**</font>)  
Enables or disables the usage of CarSim vs terrain file specified in the `.simfile`. By default this option is disabled and CarSim uses unreal engine methods to process the geometry of the scene.  
    - **Parameters:**
        - `enabled` (_bool_)  

##### Getters
- <a name="carla.Vehicle.get_control"></a>**<font color="#7fb800">get_control</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the control applied in the last tick. The method does not call the simulator.  
    - **Return:** _[carla.VehicleControl](#carla.VehicleControl)_  
- <a name="carla.Vehicle.get_light_state"></a>**<font color="#7fb800">get_light_state</font>**(<font color="#00a6ed">**self**</font>)  
Returns a flag representing the vehicle light state, this represents which lights are active or not.  
    - **Return:** _[carla.VehicleLightState](#carla.VehicleLightState)_  
    - **Setter:** _[carla.Vehicle.set_light_state](#carla.Vehicle.set_light_state)_  
- <a name="carla.Vehicle.get_physics_control"></a>**<font color="#7fb800">get_physics_control</font>**(<font color="#00a6ed">**self**</font>)  
The simulator returns the last physics control applied to this vehicle.  
    - **Return:** _[carla.VehiclePhysicsControl](#carla.VehiclePhysicsControl)_  
    - **Warning:** <font color="#ED2F2F">_This method does call the simulator to retrieve the value._</font>  
- <a name="carla.Vehicle.get_speed_limit"></a>**<font color="#7fb800">get_speed_limit</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the speed limit affecting this vehicle according to last tick (it does not call the simulator). The speed limit is updated when passing by a speed limit signal, so a vehicle might have none right after spawning.  
    - **Return:** _float<small> – m/s</small>_  
- <a name="carla.Vehicle.get_traffic_light"></a>**<font color="#7fb800">get_traffic_light</font>**(<font color="#00a6ed">**self**</font>)  
Retrieves the traffic light actor affecting this vehicle (if any) according to last tick. The method does not call the simulator.  
    - **Return:** _[carla.TrafficLight](#carla.TrafficLight)_  
- <a name="carla.Vehicle.get_traffic_light_state"></a>**<font color="#7fb800">get_traffic_light_state</font>**(<font color="#00a6ed">**self**</font>)  
The client returns the state of the traffic light affecting this vehicle according to last tick. The method does not call the simulator. If no traffic light is currently affecting the vehicle, returns <b>green</b>.  
    - **Return:** _[carla.TrafficLightState](#carla.TrafficLightState)_  

##### Setters
- <a name="carla.Vehicle.set_autopilot"></a>**<font color="#7fb800">set_autopilot</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**enabled**=True</font>, <font color="#00a6ed">**port**=8000</font>)  
Registers or deletes the vehicle from a Traffic Manager's list. When __True__, the Traffic Manager passed as parameter will move the vehicle around. The autopilot takes place client-side.  
    - **Parameters:**
        - `enabled` (_bool_)  
        - `port` (_uint16_) – The port of the TM-Server where the vehicle is to be registered or unlisted. If __None__ is passed, it will consider a TM at default port `8000`.  
- <a name="carla.Vehicle.set_light_state"></a>**<font color="#7fb800">set_light_state</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**light_state**</font>)  
Sets the light state of a vehicle using a flag that represents the lights that are on and off.  
    - **Parameters:**
        - `light_state` (_[carla.VehicleLightState](#carla.VehicleLightState)_)  
    - **Getter:** _[carla.Vehicle.get_light_state](#carla.Vehicle.get_light_state)_  

##### Dunder methods
- <a name="carla.Vehicle.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.VehicleControl<a name="carla.VehicleControl"></a>
Manages the basic movement of a vehicle using typical driving controls.  

### Instance Variables
- <a name="carla.VehicleControl.throttle"></a>**<font color="#f8805a">throttle</font>** (_float_)  
A scalar value to control the vehicle throttle [0.0, 1.0]. Default is 0.0.  
- <a name="carla.VehicleControl.steer"></a>**<font color="#f8805a">steer</font>** (_float_)  
A scalar value to control the vehicle steering [-1.0, 1.0]. Default is 0.0.  
- <a name="carla.VehicleControl.brake"></a>**<font color="#f8805a">brake</font>** (_float_)  
A scalar value to control the vehicle brake [0.0, 1.0]. Default is 0.0.  
- <a name="carla.VehicleControl.hand_brake"></a>**<font color="#f8805a">hand_brake</font>** (_bool_)  
Determines whether hand brake will be used. Default is <b>False</b>.  
- <a name="carla.VehicleControl.reverse"></a>**<font color="#f8805a">reverse</font>** (_bool_)  
Determines whether the vehicle will move backwards. Default is <b>False</b>.  
- <a name="carla.VehicleControl.manual_gear_shift"></a>**<font color="#f8805a">manual_gear_shift</font>** (_bool_)  
Determines whether the vehicle will be controlled by changing gears manually. Default is <b>False</b>.  
- <a name="carla.VehicleControl.gear"></a>**<font color="#f8805a">gear</font>** (_int_)  
States which gear is the vehicle running on.  

### Methods
- <a name="carla.VehicleControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**throttle**=0.0</font>, <font color="#00a6ed">**steer**=0.0</font>, <font color="#00a6ed">**brake**=0.0</font>, <font color="#00a6ed">**hand_brake**=False</font>, <font color="#00a6ed">**reverse**=False</font>, <font color="#00a6ed">**manual_gear_shift**=False</font>, <font color="#00a6ed">**gear**=0</font>)  
    - **Parameters:**
        - `throttle` (_float_) – Scalar value between [0.0,1.0].  
        - `steer` (_float_) – Scalar value between [0.0,1.0].  
        - `brake` (_float_) – Scalar value between [0.0,1.0].  
        - `hand_brake` (_bool_)  
        - `reverse` (_bool_)  
        - `manual_gear_shift` (_bool_)  
        - `gear` (_int_)  

##### Dunder methods
- <a name="carla.VehicleControl.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.VehicleControl](#carla.VehicleControl)</font>)  
- <a name="carla.VehicleControl.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.VehicleControl](#carla.VehicleControl)</font>)  
- <a name="carla.VehicleControl.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.Sensor<a name="carla.Sensor"></a>
<div class="Inherited"><small><b>Inherited from _[carla.Actor](#carla.Actor)_</b></small></div>Sensors compound a specific family of actors quite diverse and unique. They are normally spawned as attachment/sons of a vehicle (take a look at [carla.World](#carla.World) to learn about actor spawning). Sensors are thoroughly designed to retrieve different types of data that they are listening to. The data they receive is shaped as different subclasses inherited from [carla.SensorData](#carla.SensorData) (depending on the sensor).

  Most sensors can be divided in two groups: those receiving data on every tick (cameras, point clouds and some specific sensors) and those who only receive under certain circumstances (trigger detectors). CARLA provides a specific set of sensors and their blueprint can be found in [carla.BlueprintLibrary](#carla.BlueprintLibrary). All the information on their preferences and settlement can be found [here](ref_sensors.md), but the list of those available in CARLA so far goes as follow.
  <br><b>Receive data on every tick.</b>
  - [Depth camera](ref_sensors.md#depth-camera).
  - [Gnss sensor](ref_sensors.md#gnss-sensor).
  - [IMU sensor](ref_sensors.md#imu-sensor).
  - [Lidar raycast](ref_sensors.md#lidar-raycast-sensor).
  - [SemanticLidar raycast](ref_sensors.md#semanticlidar-raycast-sensor).
  - [Radar](ref_sensors.md#radar-sensor).
  - [RGB camera](ref_sensors.md#rgb-camera).
  - [RSS sensor](ref_sensors.md#rss-sensor).
  - [Semantic Segmentation camera](ref_sensors.md#semantic-segmentation-camera).
  <br><b>Only receive data when triggered.</b>
  - [Collision detector](ref_sensors.md#collision-detector).
  - [Lane invasion detector](ref_sensors.md#lane-invasion-detector).
  - [Obstacle detector](ref_sensors.md#obstacle-detector).  

### Instance Variables
- <a name="carla.Sensor.is_listening"></a>**<font color="#f8805a">is_listening</font>** (_boolean_)  
When <b>True</b> the sensor will be waiting for data.  

### Methods
- <a name="carla.Sensor.listen"></a>**<font color="#7fb800">listen</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**callback**</font>)<button class="SnipetButton" id="carla.Sensor.listen-snipet_button">snipet &rarr;</button>  
The function the sensor will be calling to every time a new measurement is received. This function needs for an argument containing an object type [carla.SensorData](#carla.SensorData) to work with.  
    - **Parameters:**
        - `callback` (_function_) – The called function with one argument containing the sensor data.  
- <a name="carla.Sensor.stop"></a>**<font color="#7fb800">stop</font>**(<font color="#00a6ed">**self**</font>)  
Commands the sensor to stop listening for data.  

##### Dunder methods
- <a name="carla.Sensor.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---

## carla.SensorData<a name="carla.SensorData"></a>
Base class for all the objects containing data generated by a [carla.Sensor](#carla.Sensor). This objects should be the argument of the function said sensor is listening to, in order to work with them. Each of these sensors needs for a specific type of sensor data. Hereunder is a list of the sensors and their corresponding data.<br>
  - Cameras (RGB, depth and semantic segmentation): [carla.Image](#carla.Image).<br>
  - Collision detector: [carla.CollisionEvent](#carla.CollisionEvent).<br>
  - GNSS sensor: [carla.GnssMeasurement](#carla.GnssMeasurement).<br>
  - IMU sensor: [carla.IMUMeasurement](#carla.IMUMeasurement).<br>
  - Lane invasion detector: [carla.LaneInvasionEvent](#carla.LaneInvasionEvent).<br>
  - LIDAR sensor: [carla.LidarMeasurement](#carla.LidarMeasurement).<br>
  - Obstacle detector: [carla.ObstacleDetectionEvent](#carla.ObstacleDetectionEvent).<br>
  - Radar sensor: [carla.RadarMeasurement](#carla.RadarMeasurement).<br>
  - RSS sensor: [carla.RssResponse](#carla.RssResponse).<br>
  - Semantic LIDAR sensor: [carla.SemanticLidarMeasurement](#carla.SemanticLidarMeasurement).  

### Instance Variables
- <a name="carla.SensorData.frame"></a>**<font color="#f8805a">frame</font>** (_int_)  
Frame count when the data was generated.  
- <a name="carla.SensorData.timestamp"></a>**<font color="#f8805a">timestamp</font>** (_float<small> – seconds</small>_)  
Simulation-time when the data was generated.  
- <a name="carla.SensorData.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
Sensor's transform when the data was generated.  

---

## carla.Timestamp<a name="carla.Timestamp"></a>
Class that contains time information for simulated data. This information is automatically retrieved as part of the [carla.WorldSnapshot](#carla.WorldSnapshot) the client gets on every frame, but might also be used in many other situations such as a [carla.Sensor](#carla.Sensor) retrieveing data.  

### Instance Variables
- <a name="carla.Timestamp.frame"></a>**<font color="#f8805a">frame</font>** (_int_)  
The number of frames elapsed since the simulator was launched.  
- <a name="carla.Timestamp.elapsed_seconds"></a>**<font color="#f8805a">elapsed_seconds</font>** (_float<small> – seconds</small>_)  
Simulated seconds elapsed since the beginning of the current episode.  
- <a name="carla.Timestamp.delta_seconds"></a>**<font color="#f8805a">delta_seconds</font>** (_float<small> – seconds</small>_)  
Simulated seconds elapsed since the previous frame.  
- <a name="carla.Timestamp.platform_timestamp"></a>**<font color="#f8805a">platform_timestamp</font>** (_float<small> – seconds</small>_)  
Time register of the frame at which this measurement was taken given by the OS in seconds.  

### Methods
- <a name="carla.Timestamp.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**frame**</font>, <font color="#00a6ed">**elapsed_seconds**</font>, <font color="#00a6ed">**delta_seconds**</font>, <font color="#00a6ed">**platform_timestamp**</font>)  
    - **Parameters:**
        - `frame` (_int_)  
        - `elapsed_seconds` (_float<small> – seconds</small>_)  
        - `delta_seconds` (_float<small> – seconds</small>_)  
        - `platform_timestamp` (_float<small> – seconds</small>_)  

##### Dunder methods
- <a name="carla.Timestamp.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Timestamp](#carla.Timestamp)</font>)  
- <a name="carla.Timestamp.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Timestamp](#carla.Timestamp)</font>)  
- <a name="carla.Timestamp.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  

---



## carla.World<a name="carla.World"></a>
World objects are created by the client to have a place for the simulation to happen. The world contains the map we can see, meaning the asset, not the navigation map. Navigation maps are part of the [carla.Map](#carla.Map) class. It also manages the weather and actors present in it. There can only be one world per simulation, but it can be changed anytime.  

### Instance Variables
- <a name="carla.World.id"></a>**<font color="#f8805a">id</font>** (_int_)  
The ID of the episode associated with this world. Episodes are different sessions of a simulation. These change everytime a world is disabled or reloaded. Keeping track is useful to avoid possible issues.  
- <a name="carla.World.debug"></a>**<font color="#f8805a">debug</font>** (_[carla.DebugHelper](#carla.DebugHelper)_)  
Responsible for creating different shapes for debugging. Take a look at its class to learn more about it.  

### Methods
- <a name="carla.World.apply_settings"></a>**<font color="#7fb800">apply_settings</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**world_settings**</font>)  
This method applies settings contained in an object to the simulation running and returns the ID of the frame they were implemented.  
    - **Parameters:**
        - `world_settings` (_[carla.WorldSettings](#carla.WorldSettings)_)  
    - **Return:** _int_  
    - **Warning:** <font color="#ED2F2F">_If synchronous mode is enabled, and there is a Traffic Manager running, this must be set to sync mode too. Read [this](adv_traffic_manager.md#synchronous-mode) to learn how to do it. 
_</font>  
- <a name="carla.World.cast_ray"></a>**<font color="#7fb800">cast_ray</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**initial_location**</font>, <font color="#00a6ed">**final_location**</font>)  
Casts a ray from the specified initial_location to final_location. The function then detects all geometries intersecting the ray and returns a list of [carla.LabelledPoint](#carla.LabelledPoint) in order.  
    - **Parameters:**
        - `initial_location` (_[carla.Location](#carla.Location)_) – The initial position of the ray.  
        - `final_location` (_[carla.Location](#carla.Location)_) – The final position of the ray.  
    - **Return:** _list([carla.LabelledPoint](#carla.LabelledPoint))_  
- <a name="carla.World.enable_environment_objects"></a>**<font color="#7fb800">enable_environment_objects</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**env_objects_ids**</font>, <font color="#00a6ed">**enable**</font>)  
Enable or disable a set of EnvironmentObject identified by their id. These objects will appear or disappear from the level.  
    - **Parameters:**
        - `env_objects_ids` (_set(int)_) – Set of EnvironmentObject ids to change.  
        - `enable` (_bool_) – State to be applied to all the EnvironmentObject of the set.  
- <a name="carla.World.freeze_all_traffic_lights"></a>**<font color="#7fb800">freeze_all_traffic_lights</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**frozen**</font>)  
Freezes or unfreezes all traffic lights in the scene. Frozen traffic lights can be modified by the user but the time will not update them until unfrozen.  
    - **Parameters:**
        - `frozen` (_bool_)  
- <a name="carla.World.ground_projection"></a>**<font color="#7fb800">ground_projection</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**search_distance**</font>)  
Projects the specified point downwards in the scene. The functions casts a ray from location in the direction (0,0,-1) (downwards) and returns a [carla.Labelled](#carla.Labelled) object with the first geometry this ray intersects (usually the ground). If no geometry is found in the search_distance range the function returns `None`.  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)_) – The point to be projected.  
        - `search_distance` (_float_) – The maximum distance to perform the projection.  
    - **Return:** _[carla.LabelledPoint](#carla.LabelledPoint)_  
- <a name="carla.World.load_map_layer"></a>**<font color="#7fb800">load_map_layer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**map_layers**</font>)  
Loads the selected layers to the level. If the layer is already loaded the call has no effect.  
    - **Parameters:**
        - `map_layers` (_[carla.MapLayer](#carla.MapLayer)_) – Mask of level layers to be loaded.  
    - **Warning:** <font color="#ED2F2F">_This only affects "Opt" maps._</font>  
- <a name="carla.World.on_tick"></a>**<font color="#7fb800">on_tick</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**callback**</font>)  
This method is used in [__asynchronous__ mode](https://[carla.readthedocs.io](#carla.readthedocs.io)/en/latest/adv_synchrony_timestep/). It starts callbacks from the client for the function defined as `callback`, and returns the ID of the callback. The function will be called everytime the server ticks. It requires a [carla.WorldSnapshot](#carla.WorldSnapshot) as argument, which can be retrieved from __<font color="#7fb800">wait_for_tick()</font>__. Use __<font color="#7fb800">remove_on_tick()</font>__ to stop the callbacks.  
    - **Parameters:**
        - `callback` (_[carla.WorldSnapshot](#carla.WorldSnapshot)_) – Function with a snapshot as compulsory parameter that will be called when the client receives a tick.  
    - **Return:** _int_  
- <a name="carla.World.project_point"></a>**<font color="#7fb800">project_point</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**direction**</font>, <font color="#00a6ed">**search_distance**</font>)  
Projects the specified point to the desired direction in the scene. The functions casts a ray from location in a direction and returns a [carla.Labelled](#carla.Labelled) object with the first geometry this ray intersects. If no geometry is found in the search_distance range the function returns `None`.  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)_) – The point to be projected.  
        - `direction` (_[carla.Vector3D](#carla.Vector3D)_) – The direction of projection.  
        - `search_distance` (_float_) – The maximum distance to perform the projection.  
    - **Return:** _[carla.LabelledPoint](#carla.LabelledPoint)_  
- <a name="carla.World.remove_on_tick"></a>**<font color="#7fb800">remove_on_tick</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**callback_id**</font>)  
Stops the callback for `callback_id` started with __<font color="#7fb800">on_tick()</font>__.  
    - **Parameters:**
        - `callback_id` (_callback_) – The callback to be removed. The ID is returned when creating the callback.  
- <a name="carla.World.reset_all_traffic_lights"></a>**<font color="#7fb800">reset_all_traffic_lights</font>**(<font color="#00a6ed">**self**</font>)  
Resets the cycle of all traffic lights in the map to the initial state.  
- <a name="carla.World.spawn_actor"></a>**<font color="#7fb800">spawn_actor</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**blueprint**</font>, <font color="#00a6ed">**transform**</font>, <font color="#00a6ed">**attach_to**=None</font>, <font color="#00a6ed">**attachment**=Rigid</font>)<button class="SnipetButton" id="carla.World.spawn_actor-snipet_button">snipet &rarr;</button>  
The method will create, return and spawn an actor into the world. The actor will need an available blueprint to be created and a transform (location and rotation). It can also be attached to a parent with a certain attachment type.  
    - **Parameters:**
        - `blueprint` (_[carla.ActorBlueprint](#carla.ActorBlueprint)_) – The reference from which the actor will be created.  
        - `transform` (_[carla.Transform](#carla.Transform)_) – Contains the location and orientation the actor will be spawned with.  
        - `attach_to` (_[carla.Actor](#carla.Actor)_) – The parent object that the spawned actor will follow around.  
        - `attachment` (_[carla.AttachmentType](#carla.AttachmentType)_) – Determines how fixed and rigorous should be the changes in position according to its parent object.  
    - **Return:** _[carla.Actor](#carla.Actor)_  
- <a name="carla.World.tick"></a>**<font color="#7fb800">tick</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**seconds**=10.0</font>)  
This method is used in [__synchronous__ mode](https://[carla.readthedocs.io](#carla.readthedocs.io)/en/latest/adv_synchrony_timestep/), when the server waits for a client tick before computing the next frame. This method will send the tick, and give way to the server. It returns the ID of the new frame computed by the server.  
    - **Parameters:**
        - `seconds` (_float<small> – seconds</small>_) – Maximum time the server should wait for a tick. It is set to <code>10.0</code> by default.  
    - **Return:** _int_  
    - **Note:** <font color="#8E8E8E">_If no tick is received in synchronous mode, the simulation will freeze. Also, if many ticks are received from different clients, there may be synchronization issues. Please read the docs about [synchronous mode](https://[carla.readthedocs.io](#carla.readthedocs.io)/en/latest/adv_synchrony_timestep/) to learn more.  
_</font>  
- <a name="carla.World.try_spawn_actor"></a>**<font color="#7fb800">try_spawn_actor</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**blueprint**</font>, <font color="#00a6ed">**transform**</font>, <font color="#00a6ed">**attach_to**=None</font>, <font color="#00a6ed">**attachment**=Rigid</font>)  
Same as __<font color="#7fb800">spawn_actor()</font>__ but returns <b>None</b> on failure instead of throwing an exception.  
    - **Parameters:**
        - `blueprint` (_[carla.ActorBlueprint](#carla.ActorBlueprint)_) – The reference from which the actor will be created.  
        - `transform` (_[carla.Transform](#carla.Transform)_) – Contains the location and orientation the actor will be spawned with.  
        - `attach_to` (_[carla.Actor](#carla.Actor)_) – The parent object that the spawned actor will follow around.  
        - `attachment` (_[carla.AttachmentType](#carla.AttachmentType)_) – Determines how fixed and rigorous should be the changes in position according to its parent object.  
    - **Return:** _[carla.Actor](#carla.Actor)_  
- <a name="carla.World.unload_map_layer"></a>**<font color="#7fb800">unload_map_layer</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**map_layers**</font>)  
Unloads the selected layers to the level. If the layer is already unloaded the call has no effect.  
    - **Parameters:**
        - `map_layers` (_[carla.MapLayer](#carla.MapLayer)_) – Mask of level layers to be unloaded.  
    - **Warning:** <font color="#ED2F2F">_This only affects "Opt" maps._</font>  
- <a name="carla.World.wait_for_tick"></a>**<font color="#7fb800">wait_for_tick</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**seconds**=10.0</font>)  
This method is used in [__asynchronous__ mode](https://[carla.readthedocs.io](#carla.readthedocs.io)/en/latest/adv_synchrony_timestep/). It makes the client wait for a server tick. When the next frame is computed, the server will tick and return a snapshot describing the new state of the world.  
    - **Parameters:**
        - `seconds` (_float<small> – seconds</small>_) – Maximum time the server should wait for a tick. It is set to <code>10.0</code> by default.  
    - **Return:** _[carla.WorldSnapshot](#carla.WorldSnapshot)_  

##### Getters
- <a name="carla.World.get_actor"></a>**<font color="#7fb800">get_actor</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor_id**</font>)  
Looks up for an actor by ID and returns <b>None</b> if not found.  
    - **Parameters:**
        - `actor_id` (_int_)  
    - **Return:** _[carla.Actor](#carla.Actor)_  
- <a name="carla.World.get_actors"></a>**<font color="#7fb800">get_actors</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor_ids**=None</font>)  
Retrieves a list of [carla.Actor](#carla.Actor) elements, either using a list of IDs provided or just listing everyone on stage. If an ID does not correspond with any actor, it will be excluded from the list returned, meaning that both the list of IDs and the list of actors may have different lengths.  
    - **Parameters:**
        - `actor_ids` (_list_) – The IDs of the actors being searched. By default it is set to <b>None</b> and returns every actor on scene.  
    - **Return:** _[carla.ActorList](#carla.ActorList)_  
- <a name="carla.World.get_blueprint_library"></a>**<font color="#7fb800">get_blueprint_library</font>**(<font color="#00a6ed">**self**</font>)  
Returns a list of actor blueprints available to ease the spawn of these into the world.  
    - **Return:** _[carla.BlueprintLibrary](#carla.BlueprintLibrary)_  
- <a name="carla.World.get_environment_objects"></a>**<font color="#7fb800">get_environment_objects</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**object_type**=Any</font>)  
Returns a list of EnvironmentObject with the requested semantic tag.  The method returns all the EnvironmentObjects in the level by default, but the query can be filtered by semantic tags with the argument `object_type`.  
    - **Parameters:**
        - `object_type` (_[carla.CityObjectLabel](#carla.CityObjectLabel)_) – Semantic tag of the EnvironmentObjects that are returned.  
    - **Return:** _array([carla.EnvironmentObject](#carla.EnvironmentObject))_  
- <a name="carla.World.get_level_bbs"></a>**<font color="#7fb800">get_level_bbs</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor_type**=Any</font>)  
Returns an array of bounding boxes with location and rotation in world space. The method returns all the bounding boxes in the level by default, but the query can be filtered by semantic tags with the argument `actor_type`.  
    - **Parameters:**
        - `actor_type` (_[carla.CityObjectLabel](#carla.CityObjectLabel)_) – Semantic tag of the elements contained in the bounding boxes that are returned.  
    - **Return:** _array([carla.BoundingBox](#carla.BoundingBox))_  
- <a name="carla.World.get_lightmanager"></a>**<font color="#7fb800">get_lightmanager</font>**(<font color="#00a6ed">**self**</font>)  
Returns an instance of [carla.LightManager](#carla.LightManager) that can be used to handle the lights in the scene.  
    - **Return:** _[carla.LightManager](#carla.LightManager)_  
- <a name="carla.World.get_map"></a>**<font color="#7fb800">get_map</font>**(<font color="#00a6ed">**self**</font>)  
Asks the server for the XODR containing the map file, and returns this parsed as a [carla.Map](#carla.Map).  
    - **Return:** _[carla.Map](#carla.Map)_  
    - **Warning:** <font color="#ED2F2F">_This method does call the simulation. It is expensive, and should only be called once.  
_</font>  
- <a name="carla.World.get_random_location_from_navigation"></a>**<font color="#7fb800">get_random_location_from_navigation</font>**(<font color="#00a6ed">**self**</font>)  
This can only be used with walkers. It retrieves a random location to be used as a destination using the __<font color="#7fb800">go_to_location()</font>__ method in [carla.WalkerAIController](#carla.WalkerAIController). This location will be part of a sidewalk. Roads, crosswalks and grass zones are excluded. The method does not take into consideration locations of existing actors so if a collision happens when trying to spawn an actor, it will return an error. Take a look at [`spawn_npc.py`](https://github.com/carla-simulator/carla/blob/e73ad54d182e743b50690ca00f1709b08b16528c/PythonAPI/examples/spawn_npc.py#L179) for an example.  
    - **Return:** _[carla.Location](#carla.Location)_  
- <a name="carla.World.get_settings"></a>**<font color="#7fb800">get_settings</font>**(<font color="#00a6ed">**self**</font>)  
Returns an object containing some data about the simulation such as synchrony between client and server or rendering mode.  
    - **Return:** _[carla.WorldSettings](#carla.WorldSettings)_  
- <a name="carla.World.get_snapshot"></a>**<font color="#7fb800">get_snapshot</font>**(<font color="#00a6ed">**self**</font>)  
Returns a snapshot of the world at a certain moment comprising all the information about the actors.  
    - **Return:** _[carla.WorldSnapshot](#carla.WorldSnapshot)_  
- <a name="carla.World.get_spectator"></a>**<font color="#7fb800">get_spectator</font>**(<font color="#00a6ed">**self**</font>)<button class="SnipetButton" id="carla.World.get_spectator-snipet_button">snipet &rarr;</button>  
Returns the spectator actor. The spectator is a special type of actor created by Unreal Engine, usually with ID=0, that acts as a camera and controls the view in the simulator window.  
    - **Return:** _[carla.Actor](#carla.Actor)_  
- <a name="carla.World.get_traffic_light"></a>**<font color="#7fb800">get_traffic_light</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**landmark**</font>)  
Provided a landmark, returns the traffic light object it describes.  
    - **Parameters:**
        - `landmark` (_[carla.Landmark](#carla.Landmark)_) – The landmark object describing a traffic light.  
    - **Return:** _[carla.TrafficLight](#carla.TrafficLight)_  
- <a name="carla.World.get_traffic_sign"></a>**<font color="#7fb800">get_traffic_sign</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**landmark**</font>)  
Provided a landmark, returns the traffic sign object it describes.  
    - **Parameters:**
        - `landmark` (_[carla.Landmark](#carla.Landmark)_) – The landmark object describing a traffic sign.  
    - **Return:** _[carla.TrafficSign](#carla.TrafficSign)_  
- <a name="carla.World.get_vehicles_light_states"></a>**<font color="#7fb800">get_vehicles_light_states</font>**(<font color="#00a6ed">**self**</font>)  
Returns a dict where the keys are [carla.Actor](#carla.Actor) IDs and the values are [carla.VehicleLightState](#carla.VehicleLightState) of that vehicle.  
    - **Return:** _dict_  
- <a name="carla.World.get_weather"></a>**<font color="#7fb800">get_weather</font>**(<font color="#00a6ed">**self**</font>)  
Retrieves an object containing weather parameters currently active in the simulation, mainly cloudiness, precipitation, wind and sun position.  
    - **Return:** _[carla.WeatherParameters](#carla.WeatherParameters)_  
    - **Setter:** _[carla.World.set_weather](#carla.World.set_weather)_  

##### Setters
- <a name="carla.World.set_weather"></a>**<font color="#7fb800">set_weather</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**weather**</font>)  
Changes the weather parameteres ruling the simulation to another ones defined in an object.  
    - **Parameters:**
        - `weather` (_[carla.WeatherParameters](#carla.WeatherParameters)_) – New conditions to be applied.  
    - **Getter:** _[carla.World.get_weather](#carla.World.get_weather)_  

##### Dunder methods
- <a name="carla.World.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
The content of the world is parsed and printed as a brief report of its current state.  
    - **Return:** _string_  

---

## carla.WorldSettings<a name="carla.WorldSettings"></a>
The simulation has some advanced configuration options that are contained in this class and can be managed using [carla.World](#carla.World) and its methods. These allow the user to choose between client-server synchrony/asynchrony, activation of "no rendering mode" and either if the simulation should run with a fixed or variable time-step. Check [this](adv_synchrony_timestep.md) out if you want to learn about it.  

### Instance Variables
- <a name="carla.WorldSettings.synchronous_mode"></a>**<font color="#f8805a">synchronous_mode</font>** (_bool_)  
States the synchrony between client and server. When set to true, the server will wait for a client tick in order to move forward. It is false by default.  
- <a name="carla.WorldSettings.no_rendering_mode"></a>**<font color="#f8805a">no_rendering_mode</font>** (_bool_)  
When enabled, the simulation will run no rendering at all. This is mainly used to avoid overhead during heavy traffic simulations. It is false by default.  
- <a name="carla.WorldSettings.fixed_delta_seconds"></a>**<font color="#f8805a">fixed_delta_seconds</font>** (_float_)  
Ensures that the time elapsed between two steps of the simulation is fixed. Set this to <b>0.0</b> to work with a variable time-step, as happens by default.  
- <a name="carla.WorldSettings.substepping"></a>**<font color="#f8805a">substepping</font>** (_bool_)  
Enable the physics substepping. This option allows computing some physics substeps between two render frames. If synchronous mode is set, the number of substeps and its time interval are fixed and computed are so they fulfilled the requirements of [carla.WorldSettings.max_substep](#carla.WorldSettings.max_substep) and [carla.WorldSettings.max_substep_delta_time](#carla.WorldSettings.max_substep_delta_time). These last two parameters need to be compatible with [carla.WorldSettings.fixed_delta_seconds](#carla.WorldSettings.fixed_delta_seconds). Enabled by default.  
- <a name="carla.WorldSettings.max_substep_delta_time"></a>**<font color="#f8805a">max_substep_delta_time</font>** (_float_)  
Maximum delta time of the substeps. If the [carla.WorldSettingsmax_substep](#carla.WorldSettingsmax_substep) is high enough, the substep delta time would be always below or equal to this value. By default, the value is set to 0.01.  
- <a name="carla.WorldSettings.max_substeps"></a>**<font color="#f8805a">max_substeps</font>** (_int_)  
The maximum number of physics substepping that are allowed. By default, the value is set to 10.  
- <a name="carla.WorldSettings.max_culling_distance"></a>**<font color="#f8805a">max_culling_distance</font>** (_float_)  
Configure the max draw distance for each mesh of the level.  
- <a name="carla.WorldSettings.deterministic_ragdolls"></a>**<font color="#f8805a">deterministic_ragdolls</font>** (_bool_)  
Defines wether to use deterministic physics for pedestrian death animations or physical ragdoll simulation.  When enabled, pedestrians have less realistic death animation but ensures determinism.  When disabled, pedestrians are simulated as ragdolls with more realistic simulation and collision but no determinsm can be ensured.  

### Methods
- <a name="carla.WorldSettings.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**synchronous_mode**=False</font>, <font color="#00a6ed">**no_rendering_mode**=False</font>, <font color="#00a6ed">**fixed_delta_seconds**=0.0</font>)  
Creates an object containing desired settings that could later be applied through [carla.World](#carla.World) and its method __<font color="#7fb800">apply_settings()</font>__.  
    - **Parameters:**
        - `synchronous_mode` (_bool_) – Set this to true to enable client-server synchrony.  
        - `no_rendering_mode` (_bool_) – Set this to true to completely disable rendering in the simulation.  
        - `fixed_delta_seconds` (_float<small> – seconds</small>_) – Set a fixed time-step in between frames. <code>0.0</code> means variable time-step and it is the default mode.  

##### Dunder methods
- <a name="carla.WorldSettings.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WorldSettings](#carla.WorldSettings)</font>)  
Returns <b>True</b> if both objects' variables are the same.  
    - **Return:** _bool_  
- <a name="carla.WorldSettings.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.WorldSettings](#carla.WorldSettings)</font>)  
Returns <b>True</b> if both objects' variables are different.  
    - **Return:** _bool_  
- <a name="carla.WorldSettings.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Parses the established settings to a string and shows them in command line.  
    - **Return:** _str_  

---


## command.ApplyAngularImpulse<a name="command.ApplyAngularImpulse"></a>
Command adaptation of __<font color="#7fb800">add_angular_impulse()</font>__ in [carla.Actor](#carla.Actor). Applies an angular impulse to an actor.  

### Instance Variables
- <a name="command.ApplyAngularImpulse.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyAngularImpulse.impulse"></a>**<font color="#f8805a">impulse</font>** (_[carla.Vector3D](#carla.Vector3D)<small> – degrees*s</small>_)  
Angular impulse applied to the actor.  

### Methods
- <a name="command.ApplyAngularImpulse.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**impulse**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `impulse` (_[carla.Vector3D](#carla.Vector3D)<small> – degrees*s</small>_)  

---

## command.ApplyForce<a name="command.ApplyForce"></a>
Command adaptation of __<font color="#7fb800">add_force()</font>__ in [carla.Actor](#carla.Actor). Applies a force to an actor.  

### Instance Variables
- <a name="command.ApplyForce.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyForce.force"></a>**<font color="#f8805a">force</font>** (_[carla.Vector3D](#carla.Vector3D)<small> – N</small>_)  
Force applied to the actor over time.  

### Methods
- <a name="command.ApplyForce.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**force**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `force` (_[carla.Vector3D](#carla.Vector3D)<small> – N</small>_)  

---

## command.ApplyImpulse<a name="command.ApplyImpulse"></a>
Command adaptation of __<font color="#7fb800">add_impulse()</font>__ in [carla.Actor](#carla.Actor). Applies an impulse to an actor.  

### Instance Variables
- <a name="command.ApplyImpulse.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyImpulse.impulse"></a>**<font color="#f8805a">impulse</font>** (_[carla.Vector3D](#carla.Vector3D)<small> – N*s</small>_)  
Impulse applied to the actor.  

### Methods
- <a name="command.ApplyImpulse.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**impulse**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `impulse` (_[carla.Vector3D](#carla.Vector3D)<small> – N*s</small>_)  

---

## command.ApplyTargetAngularVelocity<a name="command.ApplyTargetAngularVelocity"></a>
Command adaptation of __<font color="#7fb800">set_target_angular_velocity()</font>__ in [carla.Actor](#carla.Actor). Sets the actor's angular velocity vector.  

### Instance Variables
- <a name="command.ApplyTargetAngularVelocity.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyTargetAngularVelocity.angular_velocity"></a>**<font color="#f8805a">angular_velocity</font>** (_[carla.Vector3D](#carla.Vector3D)<small> – deg/s</small>_)  
The 3D angular velocity that will be applied to the actor.  

### Methods
- <a name="command.ApplyTargetAngularVelocity.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**angular_velocity**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `angular_velocity` (_[carla.Vector3D](#carla.Vector3D)<small> – deg/s</small>_) – Angular velocity vector applied to the actor.  

---

## command.ApplyTargetVelocity<a name="command.ApplyTargetVelocity"></a>
Command adaptation of __<font color="#7fb800">set_target_velocity()</font>__ in [carla.Actor](#carla.Actor).  

### Instance Variables
- <a name="command.ApplyTargetVelocity.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyTargetVelocity.velocity"></a>**<font color="#f8805a">velocity</font>** (_[carla.Vector3D](#carla.Vector3D)<small> – m/s</small>_)  
The 3D velocity applied to the actor.  

### Methods
- <a name="command.ApplyTargetVelocity.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**velocity**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `velocity` (_[carla.Vector3D](#carla.Vector3D)<small> – m/s</small>_) – Velocity vector applied to the actor.  

---

## command.ApplyTorque<a name="command.ApplyTorque"></a>
Command adaptation of __<font color="#7fb800">add_torque()</font>__ in [carla.Actor](#carla.Actor). Applies a torque to an actor.  

### Instance Variables
- <a name="command.ApplyTorque.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyTorque.torque"></a>**<font color="#f8805a">torque</font>** (_[carla.Vector3D](#carla.Vector3D)<small> – degrees</small>_)  
Torque applied to the actor over time.  

### Methods
- <a name="command.ApplyTorque.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**torque**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `torque` (_[carla.Vector3D](#carla.Vector3D)<small> – degrees</small>_)  

---

## command.ApplyTransform<a name="command.ApplyTransform"></a>
Command adaptation of __<font color="#7fb800">set_transform()</font>__ in [carla.Actor](#carla.Actor). Sets a new transform to an actor.  

### Instance Variables
- <a name="command.ApplyTransform.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.ApplyTransform.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
Transformation to be applied.  

### Methods
- <a name="command.ApplyTransform.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**transform**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `transform` (_[carla.Transform](#carla.Transform)_)  

---

## command.ApplyVehicleControl<a name="command.ApplyVehicleControl"></a>
Command adaptation of __<font color="#7fb800">apply_control()</font>__ in [carla.Vehicle](#carla.Vehicle). Applies a certain control to a vehicle.  

### Instance Variables
- <a name="command.ApplyVehicleControl.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Vehicle actor affected by the command.  
- <a name="command.ApplyVehicleControl.control"></a>**<font color="#f8805a">control</font>** (_[carla.VehicleControl](#carla.VehicleControl)_)  
Vehicle control to be applied.  

### Methods
- <a name="command.ApplyVehicleControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**control**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `control` (_[carla.VehicleControl](#carla.VehicleControl)_)  

---

## command.ApplyWalkerControl<a name="command.ApplyWalkerControl"></a>
Command adaptation of __<font color="#7fb800">apply_control()</font>__ in [carla.Walker](#carla.Walker). Applies a control to a walker.  

### Instance Variables
- <a name="command.ApplyWalkerControl.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Walker actor affected by the command.  
- <a name="command.ApplyWalkerControl.control"></a>**<font color="#f8805a">control</font>** (_[carla.WalkerControl](#carla.WalkerControl)_)  
Walker control to be applied.  

### Methods
- <a name="command.ApplyWalkerControl.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**control**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `control` (_[carla.WalkerControl](#carla.WalkerControl)_)  

---

## command.ApplyWalkerState<a name="command.ApplyWalkerState"></a>
Apply a state to the walker actor. Specially useful to initialize an actor them with a specific location, orientation and speed.  

### Instance Variables
- <a name="command.ApplyWalkerState.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Walker actor affected by the command.  
- <a name="command.ApplyWalkerState.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
Transform to be applied.  
- <a name="command.ApplyWalkerState.speed"></a>**<font color="#f8805a">speed</font>** (_float<small> – m/s</small>_)  
Speed to be applied.  

### Methods
- <a name="command.ApplyWalkerState.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**transform**</font>, <font color="#00a6ed">**speed**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `transform` (_[carla.Transform](#carla.Transform)_)  
        - `speed` (_float<small> – m/s</small>_)  

---

## command.DestroyActor<a name="command.DestroyActor"></a>
Command adaptation of __<font color="#7fb800">destroy()</font>__ in [carla.Actor](#carla.Actor) that tells the simulator to destroy this actor. It has no effect if the actor was already destroyed. When executed with __<font color="#7fb800">apply_batch_sync()</font>__ in [carla.Client](#carla.Client) there will be a <b>command.Response</b> that will return a boolean stating whether the actor was successfully destroyed.  

### Instance Variables
- <a name="command.DestroyActor.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  

### Methods
- <a name="command.DestroyActor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  

---

## command.Response<a name="command.Response"></a>
States the result of executing a command as either the ID of the actor to whom the command was applied to (when succeeded) or an error string (when failed).  actor ID, depending on whether or not the command succeeded. The method __<font color="#7fb800">apply_batch_sync()</font>__ in [carla.Client](#carla.Client) returns a list of these to summarize the execution of a batch.  

### Instance Variables
- <a name="command.Response.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor to whom the command was applied to. States that the command was successful.  
- <a name="command.Response.error"></a>**<font color="#f8805a">error</font>** (_str_)  
A string stating the command has failed.  

### Methods
- <a name="command.Response.has_error"></a>**<font color="#7fb800">has_error</font>**(<font color="#00a6ed">**self**</font>)  
Returns <b>True</b> if the command execution fails, and <b>False</b> if it was successful.  
    - **Return:** _bool_  

---

## command.SetAutopilot<a name="command.SetAutopilot"></a>
Command adaptation of __<font color="#7fb800">set_autopilot()</font>__ in [carla.Vehicle](#carla.Vehicle). Turns on/off the vehicle's autopilot mode.  

### Instance Variables
- <a name="command.SetAutopilot.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor that is affected by the command.  
- <a name="command.SetAutopilot.enabled"></a>**<font color="#f8805a">enabled</font>** (_bool_)  
If autopilot should be activated or not.  
- <a name="command.SetAutopilot.port"></a>**<font color="#f8805a">port</font>** (_uint16_)  
Port of the Traffic Manager where the vehicle is to be registered or unlisted.  

### Methods
- <a name="command.SetAutopilot.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**enabled**</font>, <font color="#00a6ed">**port**=8000</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `enabled` (_bool_)  
        - `port` (_uint16_) – The Traffic Manager port where the vehicle is to be registered or unlisted. If __None__ is passed, it will consider a TM at default port `8000`.  

---

## command.SetSimulatePhysics<a name="command.SetSimulatePhysics"></a>
Command adaptation of __<font color="#7fb800">set_simulate_physics()</font>__ in [carla.Actor](#carla.Actor). Determines whether an actor will be affected by physics or not.  

### Instance Variables
- <a name="command.SetSimulatePhysics.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor affected by the command.  
- <a name="command.SetSimulatePhysics.enabled"></a>**<font color="#f8805a">enabled</font>** (_bool_)  
If physics should be activated or not.  

### Methods
- <a name="command.SetSimulatePhysics.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**enabled**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `enabled` (_bool_)  

---

## command.SetVehicleLightState<a name="command.SetVehicleLightState"></a>
Command adaptation of __<font color="#7fb800">set_light_state()</font>__ in [carla.Vehicle](#carla.Vehicle). Sets the light state of a vehicle.  

### Instance Variables
- <a name="command.SetVehicleLightState.actor_id"></a>**<font color="#f8805a">actor_id</font>** (_int_)  
Actor that is affected by the command.  
- <a name="command.SetVehicleLightState.light_state"></a>**<font color="#f8805a">light_state</font>** (_[carla.VehicleLightState](#carla.VehicleLightState)_)  
Defines the light state of a vehicle.  

### Methods
- <a name="command.SetVehicleLightState.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**actor**</font>, <font color="#00a6ed">**light_state**</font>)  
    - **Parameters:**
        - `actor` (_[carla.Actor](#carla.Actor) or int_) – Actor or its ID to whom the command will be applied to.  
        - `light_state` (_[carla.VehicleLightState](#carla.VehicleLightState)_) – Recaps the state of the lights of a vehicle, these can be used as a flags.  

---

## command.SpawnActor<a name="command.SpawnActor"></a>
Command adaptation of __<font color="#7fb800">spawn_actor()</font>__ in [carla.World](#carla.World). Spawns an actor into the world based on the blueprint provided and the transform. If a parent is provided, the actor is attached to it.  

### Instance Variables
- <a name="command.SpawnActor.transform"></a>**<font color="#f8805a">transform</font>** (_[carla.Transform](#carla.Transform)_)  
Transform to be applied.  
- <a name="command.SpawnActor.parent_id"></a>**<font color="#f8805a">parent_id</font>** (_int_)  
Identificator of the parent actor.  

### Methods
- <a name="command.SpawnActor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>)  
- <a name="command.SpawnActor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**blueprint**</font>, <font color="#00a6ed">**transform**</font>)  
    - **Parameters:**
        - `blueprint` (_[carla.ActorBlueprint](#carla.ActorBlueprint)_)  
        - `transform` (_[carla.Transform](#carla.Transform)_)  
- <a name="command.SpawnActor.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**blueprint**</font>, <font color="#00a6ed">**transform**</font>, <font color="#00a6ed">**parent**</font>)  
    - **Parameters:**
        - `blueprint` (_[carla.ActorBlueprint](#carla.ActorBlueprint)_)  
        - `transform` (_[carla.Transform](#carla.Transform)_)  
        - `parent` (_[carla.Actor](#carla.Actor) or int_)  
- <a name="command.SpawnActor.then"></a>**<font color="#7fb800">then</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**command**</font>)  
Links another command to be executed right after. It allows to ease very common flows such as spawning a set of vehicles by command and then using this method to set them to autopilot automatically.  
    - **Parameters:**
        - `command` (_any carla Command_) – a Carla command.  

## carla.Vector3D<a name="carla.Vector3D"></a>
Helper class to perform 3D operations.  

### Instance Variables
- <a name="carla.Vector3D.x"></a>**<font color="#f8805a">x</font>** (_float_)  
X-axis value.  
- <a name="carla.Vector3D.y"></a>**<font color="#f8805a">y</font>** (_float_)  
Y-axis value.  
- <a name="carla.Vector3D.z"></a>**<font color="#f8805a">z</font>** (_float_)  
Z-axis value.  

### Methods
- <a name="carla.Vector3D.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**=0.0</font>, <font color="#00a6ed">**y**=0.0</font>, <font color="#00a6ed">**z**=0.0</font>)  
    - **Parameters:**
        - `x` (_float_)  
        - `y` (_float_)  
        - `z` (_float_)  

##### Dunder methods
- <a name="carla.Vector3D.__add__"></a>**<font color="#7fb800">\__add__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
- <a name="carla.Vector3D.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
Returns __True__ if values for every axis are equal.  
    - **Return:** _bool_  
- <a name="carla.Vector3D.__mul__"></a>**<font color="#7fb800">\__mul__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
- <a name="carla.Vector3D.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
Returns __True__ if the value for any axis is different.  
    - **Return:** _bool_  
- <a name="carla.Vector3D.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Returns the axis values for the vector parsed as string.  
    - **Return:** _str_  
- <a name="carla.Vector3D.__sub__"></a>**<font color="#7fb800">\__sub__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  
- <a name="carla.Vector3D.__truediv__"></a>**<font color="#7fb800">\__truediv__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Vector3D](#carla.Vector3D)</font>)  

---

## carla.Location<a name="carla.Location"></a>
<div class="Inherited"><small><b>Inherited from _[carla.Vector3D](#carla.Vector3D)_</b></small></div>Represents a spot in the world.  

### Instance Variables
- <a name="carla.Location.x"></a>**<font color="#f8805a">x</font>** (_float<small> – meters</small>_)  
Distance from origin to spot on X axis.  
- <a name="carla.Location.y"></a>**<font color="#f8805a">y</font>** (_float<small> – meters</small>_)  
Distance from origin to spot on Y axis.  
- <a name="carla.Location.z"></a>**<font color="#f8805a">z</font>** (_float<small> – meters</small>_)  
Distance from origin to spot on Z axis.  

### Methods
- <a name="carla.Location.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**=0.0</font>, <font color="#00a6ed">**y**=0.0</font>, <font color="#00a6ed">**z**=0.0</font>)  
    - **Parameters:**
        - `x` (_float_)  
        - `y` (_float_)  
        - `z` (_float_)  
- <a name="carla.Location.distance"></a>**<font color="#7fb800">distance</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>)  
Returns Euclidean distance from this location to another one.  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)_) – The other point to compute the distance with.  
    - **Return:** _float<small> – meters</small>_  

##### Dunder methods
- <a name="carla.Location.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Location](#carla.Location)</font>)  
Returns __True__ if both locations are the same point in space.  
    - **Return:** _bool_  
- <a name="carla.Location.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Location](#carla.Location)</font>)  
Returns __True__ if both locations are different points in space.  
    - **Return:** _bool_  
- <a name="carla.Location.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Parses the axis' values to string.  
    - **Return:** _str_  

---

## carla.Location<a name="carla.Location"></a>
<div class="Inherited"><small><b>Inherited from _[carla.Vector3D](#carla.Vector3D)_</b></small></div>Represents a spot in the world.  

### Instance Variables
- <a name="carla.Location.x"></a>**<font color="#f8805a">x</font>** (_float<small> – meters</small>_)  
Distance from origin to spot on X axis.  
- <a name="carla.Location.y"></a>**<font color="#f8805a">y</font>** (_float<small> – meters</small>_)  
Distance from origin to spot on Y axis.  
- <a name="carla.Location.z"></a>**<font color="#f8805a">z</font>** (_float<small> – meters</small>_)  
Distance from origin to spot on Z axis.  

### Methods
- <a name="carla.Location.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**x**=0.0</font>, <font color="#00a6ed">**y**=0.0</font>, <font color="#00a6ed">**z**=0.0</font>)  
    - **Parameters:**
        - `x` (_float_)  
        - `y` (_float_)  
        - `z` (_float_)  
- <a name="carla.Location.distance"></a>**<font color="#7fb800">distance</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>)  
Returns Euclidean distance from this location to another one.  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)_) – The other point to compute the distance with.  
    - **Return:** _float<small> – meters</small>_  

##### Dunder methods
- <a name="carla.Location.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Location](#carla.Location)</font>)  
Returns __True__ if both locations are the same point in space.  
    - **Return:** _bool_  
- <a name="carla.Location.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Location](#carla.Location)</font>)  
Returns __True__ if both locations are different points in space.  
    - **Return:** _bool_  
- <a name="carla.Location.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Parses the axis' values to string.  
    - **Return:** _str_  

---

## carla.Transform<a name="carla.Transform"></a>
Class that defines a transformation, a combination of location and rotation, without scaling.  

### Instance Variables
- <a name="carla.Transform.location"></a>**<font color="#f8805a">location</font>** (_[carla.Location](#carla.Location)_)  
Describes a point in the coordinate system.  
- <a name="carla.Transform.rotation"></a>**<font color="#f8805a">rotation</font>** (_[carla.Rotation](#carla.Rotation)<small> – degrees (pitch, yaw, roll)</small>_)  
Describes a rotation for an object according to Unreal Engine's axis system.  

### Methods
- <a name="carla.Transform.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**rotation**</font>)  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)_)  
        - `rotation` (_[carla.Rotation](#carla.Rotation)<small> – degrees (pitch, yaw, roll)</small>_)  
- <a name="carla.Transform.transform"></a>**<font color="#7fb800">transform</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**in_point**</font>)  
Translates a 3D point from local to global coordinates using the current transformation as frame of reference.  
    - **Parameters:**
        - `in_point` (_[carla.Location](#carla.Location)_) – Location in the space to which the transformation will be applied.  

##### Getters
- <a name="carla.Transform.get_forward_vector"></a>**<font color="#7fb800">get_forward_vector</font>**(<font color="#00a6ed">**self**</font>)  
Computes a forward vector using the rotation of the object.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Transform.get_inverse_matrix"></a>**<font color="#7fb800">get_inverse_matrix</font>**(<font color="#00a6ed">**self**</font>)  
Computes the 4-matrix representation of the inverse transformation.  
    - **Return:** _list(list(float))_  
- <a name="carla.Transform.get_matrix"></a>**<font color="#7fb800">get_matrix</font>**(<font color="#00a6ed">**self**</font>)  
Computes the 4-matrix representation of the transformation.  
    - **Return:** _list(list(float))_  
- <a name="carla.Transform.get_right_vector"></a>**<font color="#7fb800">get_right_vector</font>**(<font color="#00a6ed">**self**</font>)  
Computes a right vector using the rotatio of the object.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  
- <a name="carla.Transform.get_up_vector"></a>**<font color="#7fb800">get_up_vector</font>**(<font color="#00a6ed">**self**</font>)  
Computes an up vector using the rotation of the object.  
    - **Return:** _[carla.Vector3D](#carla.Vector3D)_  

##### Dunder methods
- <a name="carla.Transform.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Transform](#carla.Transform)</font>)  
Returns __True__ if both location and rotation are equal for this and `other`.  
    - **Return:** _bool_  
- <a name="carla.Transform.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.Transform](#carla.Transform)</font>)  
Returns __True__ if any location and rotation are not equal for this and `other`.  
    - **Return:** _bool_  
- <a name="carla.Transform.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Parses both location and rotation to string.  
    - **Return:** _str_  

---

## carla.BoundingBox<a name="carla.BoundingBox"></a>
Bounding boxes contain the geometry of an actor or an element in the scene. They can be used by [carla.DebugHelper](#carla.DebugHelper) or a [carla.Client](#carla.Client) to draw their shapes for debugging. Check out the snipet in [carla.DebugHelper.draw_box](#carla.DebugHelper.draw_box) where a snapshot of the world is used to draw bounding boxes for traffic lights.  

### Instance Variables
- <a name="carla.BoundingBox.extent"></a>**<font color="#f8805a">extent</font>** (_[carla.Vector3D](#carla.Vector3D)<small> – meters</small>_)  
Vector from the center of the box to one vertex. The value in each axis equals half the size of the box for that axis.
`extent.x * 2` would return the size of the box in the X-axis.  
- <a name="carla.BoundingBox.location"></a>**<font color="#f8805a">location</font>** (_[carla.Location](#carla.Location)<small> – meters</small>_)  
The center of the bounding box.  
- <a name="carla.BoundingBox.rotation"></a>**<font color="#f8805a">rotation</font>** (_[carla.Rotation](#carla.Rotation)_)  
The orientation of the bounding box.  

### Methods
- <a name="carla.BoundingBox.__init__"></a>**<font color="#7fb800">\__init__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**location**</font>, <font color="#00a6ed">**extent**</font>)  
    - **Parameters:**
        - `location` (_[carla.Location](#carla.Location)_) – Center of the box, relative to its parent.  
        - `extent` (_[carla.Vector3D](#carla.Vector3D)<small> – meters</small>_) – Vector containing half the size of the box for every axis.  
- <a name="carla.BoundingBox.contains"></a>**<font color="#7fb800">contains</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**world_point**</font>, <font color="#00a6ed">**transform**</font>)  
Returns **True** if a point passed in world space is inside this bounding box.  
    - **Parameters:**
        - `world_point` (_[carla.Location](#carla.Location)<small> – meters</small>_) – The point in world space to be checked.  
        - `transform` (_[carla.Transform](#carla.Transform)_) – Contains location and rotation needed to convert this object's local space to world space.  
    - **Return:** _bool_  

##### Getters
- <a name="carla.BoundingBox.get_local_vertices"></a>**<font color="#7fb800">get_local_vertices</font>**(<font color="#00a6ed">**self**</font>)  
Returns a list containing the locations of this object's vertices in local space.  
    - **Return:** _list([carla.Location](#carla.Location))_  
- <a name="carla.BoundingBox.get_world_vertices"></a>**<font color="#7fb800">get_world_vertices</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**transform**</font>)  
Returns a list containing the locations of this object's vertices in world space.  
    - **Parameters:**
        - `transform` (_[carla.Transform](#carla.Transform)_) – Contains location and rotation needed to convert this object's local space to world space.  
    - **Return:** _list([carla.Location](#carla.Location))_  

##### Dunder methods
- <a name="carla.BoundingBox.__eq__"></a>**<font color="#7fb800">\__eq__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.BoundingBox](#carla.BoundingBox)</font>)  
Returns true if both location and extent are equal for this and `other`.  
    - **Return:** _bool_  
- <a name="carla.BoundingBox.__ne__"></a>**<font color="#7fb800">\__ne__</font>**(<font color="#00a6ed">**self**</font>, <font color="#00a6ed">**other**=[carla.BoundingBox](#carla.BoundingBox)</font>)  
Returns true if either location or extent are different for this and `other`.  
    - **Return:** _bool_  
- <a name="carla.BoundingBox.__str__"></a>**<font color="#7fb800">\__str__</font>**(<font color="#00a6ed">**self**</font>)  
Parses the location and extent of the bounding box to string.  
    - **Return:** _str_  

---
