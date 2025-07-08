# ROS2 domain id for network communication, machines with the same ID will receive each others' messages
export ROS_DOMAIN_ID=1

WORKSPACE_DIR=$(pwd) # or set to your workspace path

export DRLNAV_BASE_PATH=$WORKSPACE_DIR/src/driveguard_drlnav

# Allow gazebo to find our turtlebot3 models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$DRLNAV_BASE_PATH/src/turtlebot3_simulations/turtlebot3_gazebo/models

# Select which turtlebot3 model we will be using (default: burger, waffle, waffle_pi)
export TURTLEBOT3_MODEL=burger

# Allow Gazebo to find the plugin for moving the obstacles
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$DRLNAV_BASE_PATH/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_drl_world/obstacle_plugin/lib

source install/setup.bash
