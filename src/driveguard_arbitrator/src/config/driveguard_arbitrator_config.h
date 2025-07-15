/**
  ******************************************************************************
  * @project        : driveguard_arbitrator
  * @file           : driveguard_arbitrator_config.h
  * @author         : 文昱 杨
  * @brief          : None
  * @attention      : None
  * @date           : 25-7-7
  ******************************************************************************
  */

#ifndef DRIVEGUARD_ARBITRATOR_CONFIG_H
#define DRIVEGUARD_ARBITRATOR_CONFIG_H

#define DEBUG_MODE             true

// SET TIME PERIOD (ms)
#define CONTROL_LOOP_PERIOD     100
#define HEARTBEAT_TIMEOUT       3000
#define SERVICE_CLIENT_TIMEOUT  200
#define MODEL_CMD_TIMEOUT       120
#define MODEL_NODE_DEACTIVATION 1000
#define MANUAL_CMD_TIMEOUT      1000
#define RESCUE_DURATION         10000   

// SET SAFETY THRESHOLD
#define SAFETY_THRESHOLD        100

// Output cmd_vel topic's name
#define CMD_VEL             "/cmd_vel"

// Input cmd_vel topics' name
#define CMD_VEL_MODEL       "/cmd_vel_model"
#define CMD_VEL_SAFE        "/cmd_vel_safe"
#define CMD_VEL_MANUAL      "/cmd_vel_manual"

// Heartbeat configuration
#define MODEL_HB_ENABLE     false
#define MODEL_HEARTBEAT     "/model_node_heartbeat"
#define SAFE_HB_ENABLE      false
#define SAFE_HEARTBEAT      "/safety_node_heartbeat"
#define MANUAL_HB_ENABLE    false
#define MANUAL_HEARTBEAT    "/manual_node_heartbeat"

// Other topics' name
#define EXT_SAFETY_ENABLE   false
#define EXT_SAFETY_RISK     "/safety_risk"
#define SET_SAFE_GOAL_POSE  "/goal_pose"
#define GOAL_STATUS         "/navigate_to_pose/_action/status"
#define LOCAL_COSTMAP       "/local_costmap/costmap_raw"
#define GLOBAL_COSTMAP      "/global_costmap/costmap_raw"

// Here to set original location coordinate
#define FRAME_ID        "map"
#define POSITION_X      2.0
#define POSITION_Y      0.0
#define POSITION_Z      0.0
#define ORI_X           0.0
#define ORI_Y           0.0
#define ORI_Z           0.0
// #define ORI_W           1.0

// Here to set rescue speed (ABS VALUE, m/s)
#define RESCUE_SPEED    0.05

#endif //DRIVEGUARD_ARBITRATOR_CONFIG_H
