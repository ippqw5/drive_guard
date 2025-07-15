/**
  ******************************************************************************
  * @project        : driveguard_arbitrator
  * @file           : driveguard_arbitrator_node.cpp
  * @author         : 文昱 杨
  * @brief          : driveguard_arbitrator rclcpp node impl file
  * @attention      : None
  * @date           : 25-6-11
  ******************************************************************************
  */

#include "driveguard_arbitrator_node.h"

namespace sml = boost::sml;

class StateMachine {
public:
    // Here is public functions
    int get_status() {return this->status;}
    void set_safety_risk(bool input) {this->safety_risk = input;}
    void set_model_alive(bool input) {this->model_alive = input;}
    void set_safety_alive(bool input) {this->safety_alive = input;}
    void set_manual_alive(bool input) {this->manual_alive = input;}

    // Here is public triggers (events)
    struct start {};        // N/A
    struct shutdown {};     // N/A
    struct model_cmd_active {};     // Trigger on receiving cmd_vel from model node
    struct model_cmd_timeout {};     // Trigger when model command times out
    struct model_node_not_online {}; // Trigger when model node is not online
    struct model_cmd_resumed {};     // Trigger when model command is resumed
    // struct model_cmd_finished {};    // Trigger when model command is finished
    struct safety_node_revived {};   // Trigger on safety node revival
    struct safety_node_not_online {}; // Trigger when safety node is not online
    struct manual_cmd_received {};   // Trigger on receiving manual command
    struct manual_cmd_terminated {}; // Trigger on manual command termination
    struct safety_risk_detected {};  // Trigger when safety risk is detected
    struct destination_reached {};   // Trigger when destination is reached
    struct vehicle_not_moving {};    // Trigger when vehicle is not moving
    struct rescue_completed {};      // Trigger when rescue is completed
    struct rescue_failed {};         // Trigger when rescue has failed

    // arbitrator statemachine
    struct sm_arbitrator {
        sm_arbitrator(StateMachine& node) : node_(node) {}
        auto operator()() const {
            using namespace sml;

            // guards remapping
            auto model_node_online = [this](auto) {return node_.model_node_online();};
            auto safety_node_online = [this](auto) {return node_.safety_node_online();};
            auto manual_node_online = [this](auto) {return node_.manual_node_online();};
            // auto safety_risk = [this](auto) {return node_.safety_risk_check();};

            // on_enter set status
            auto enter_IDLE = [this](auto) {node_.set_status(0);};
            auto enter_MODEL_OK = [this](auto) {node_.set_status(1);};
            auto enter_MODEL_TIMEOUT = [this](auto) {node_.set_status(2);};
            auto enter_SAFETY_OVERRIDE = [this](auto) {node_.set_status(3);};
            auto enter_SAFETY_FAILURE = [this](auto) {node_.set_status(4);};
            auto enter_MANUAL_OVERRIDE = [this](auto) {node_.set_status(5);};
            auto enter_STOP = [this](auto) {node_.set_status(6);};

            // transition_table
            return make_transition_table(
                    * "IDLE"_s + event<model_cmd_active> [model_node_online&&safety_node_online] / enter_MODEL_OK = "MODEL_OK"_s
                    , "IDLE"_s + event<safety_node_not_online> / enter_STOP = "STOP"_s
                    , "IDLE"_s + event<manual_cmd_received> [manual_node_online] / enter_MANUAL_OVERRIDE = "MANUAL_OVERRIDE"_s
                    // , "IDLE"_s + event<safety_risk_detected> [safety_node_online] / enter_SAFETY_OVERRIDE  = "SAFETY_OVERRIDE"_s
                    , "IDLE"_s + event<shutdown>  = X
                    // , "MODEL_OK"_s + event<model_cmd_finished> / enter_IDLE  = "IDLE"_s
                    , "MODEL_OK"_s + event<safety_node_not_online> / enter_STOP  = "STOP"_s
                    , "MODEL_OK"_s + event<manual_cmd_received> [manual_node_online] / enter_MANUAL_OVERRIDE  = "MANUAL_OVERRIDE"_s
                    , "MODEL_OK"_s + event<safety_risk_detected> [safety_node_online] / enter_SAFETY_OVERRIDE  = "SAFETY_OVERRIDE"_s
                    , "MODEL_OK"_s + event<model_cmd_timeout> [model_node_online&&safety_node_online] / enter_MODEL_TIMEOUT  = "MODEL_TIMEOUT"_s
                    , "MODEL_TIMEOUT"_s + event<model_node_not_online> [safety_node_online] / enter_SAFETY_OVERRIDE  = "SAFETY_OVERRIDE"_s
                    , "MODEL_TIMEOUT"_s + event<model_cmd_resumed> [model_node_online&&safety_node_online] / enter_MODEL_OK  = "MODEL_OK"_s
                    , "MODEL_TIMEOUT"_s + event<manual_cmd_received> [manual_node_online] / enter_MANUAL_OVERRIDE  = "MANUAL_OVERRIDE"_s
                    , "MODEL_TIMEOUT"_s + event<safety_node_not_online> / enter_STOP  = "STOP"_s
                    , "MODEL_TIMEOUT"_s + event<safety_risk_detected> / enter_SAFETY_OVERRIDE  = "SAFETY_OVERRIDE"_s
                    , "SAFETY_OVERRIDE"_s + event<manual_cmd_received> [manual_node_online] / enter_MANUAL_OVERRIDE  = "MANUAL_OVERRIDE"_s
                    , "SAFETY_OVERRIDE"_s + event<destination_reached> / enter_IDLE  = "IDLE"_s
                    , "SAFETY_OVERRIDE"_s + event<safety_node_not_online> / enter_STOP  = "STOP"_s
                    , "SAFETY_OVERRIDE"_s + event<vehicle_not_moving> [safety_node_online] / enter_SAFETY_FAILURE  = "SAFETY_FAILURE"_s
                    , "SAFETY_FAILURE"_s + event<rescue_completed> [safety_node_online] / enter_SAFETY_OVERRIDE  = "SAFETY_OVERRIDE"_s
                    , "SAFETY_FAILURE"_s + event<safety_node_not_online> / enter_STOP  = "STOP"_s
                    , "SAFETY_FAILURE"_s + event<manual_cmd_received> [manual_node_online] / enter_MANUAL_OVERRIDE  = "MANUAL_OVERRIDE"_s
                    , "SAFETY_FAILURE"_s + event<rescue_failed> [safety_node_online] / enter_IDLE  = "IDLE"_s
                    , "STOP"_s + event<safety_node_revived> [safety_node_online] / enter_IDLE  = "IDLE"_s
                    , "STOP"_s + event<manual_cmd_received> [manual_node_online] / enter_MANUAL_OVERRIDE  = "MANUAL_OVERRIDE"_s
                    , "MANUAL_OVERRIDE"_s + event<manual_cmd_terminated> [safety_node_online] / enter_IDLE  = "IDLE"_s
                    , "MANUAL_OVERRIDE"_s + event<manual_cmd_terminated> [(!safety_node_online)] / enter_STOP  = "STOP"_s
            );
        }
        StateMachine& node_;
    };

private:
    // Here is public variables
    bool model_node_online() const { return this->model_alive; }
    bool safety_node_online() const { return this->safety_alive; }
    bool manual_node_online() {return this->manual_alive;}
    bool safety_risk_check()  { return this->safety_risk; }

    // heartbeat status
    bool model_alive = true;
    bool safety_alive = true;
    bool manual_alive = false;

    // safety detection
    bool safety_risk = false;

    // status
    int status = 0;
    void set_status(int input) {
        this->status = input;
    }
};

using namespace std::placeholders;
class CmdVelArbitratorNode : public rclcpp::Node {
public:
    CmdVelArbitratorNode() : Node("driveguard_arbitrator_node"), sm_(StateMachine::sm_arbitrator(statemachine)) {
        /* MAIN TOPICS */
        // output: cmd_vel
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(CMD_VEL, 10);

        // cmd_vel input: cmd_vel_model, cmd_vel_safe, cmd_vel_manual
        model_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                CMD_VEL_MODEL, 10, std::bind(&CmdVelArbitratorNode::model_callback, this, std::placeholders::_1)
        );

        safety_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                CMD_VEL_SAFE, 10, std::bind(&CmdVelArbitratorNode::safety_callback, this, std::placeholders::_1)
        );

        manual_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                CMD_VEL_MANUAL, 10, std::bind(&CmdVelArbitratorNode::manual_callback, this, std::placeholders::_1)
        );

        /* HEARTBEAT TOPICS */
        #if MODEL_HB_ENABLE
        model_heartbeat_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                MODEL_HEARTBEAT, 10, std::bind(&CmdVelArbitratorNode::model_alive_callback, this, std::placeholders::_1)
                );
        #endif
        #if SAFE_HB_ENABLE
        safety_heartbeat_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                SAFE_HEARTBEAT, 10, std::bind(&CmdVelArbitratorNode::safety_alive_callback, this, std::placeholders::_1)
                );
        #endif
        #if MANUAL_HB_ENABLE
        manual_heartbeat_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                MANUAL_HEARTBEAT, 10, std::bind(&CmdVelArbitratorNode::manual_alive_callback, this, std::placeholders::_1)
                );
        #endif

        /* OTHER TOPICS */
        // set goal_pose
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(SET_SAFE_GOAL_POSE, 10);

        // check external safety risk
        #if EXT_SAFETY_ENABLE
        safety_risk_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                EXT_SAFETY_RISK, 10, std::bind(&CmdVelArbitratorNode::safety_risk_callback, this, _1)
        );
        #endif

        // check SAFETY_CONTROLLER status
        safety_goal_status_sub_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
                GOAL_STATUS, 10, std::bind(&CmdVelArbitratorNode::safety_status_callback, this, _1)
        );

        // local costmap
        local_costmap_sub_ = this->create_subscription<nav2_msgs::msg::Costmap>(
                LOCAL_COSTMAP, 10, std::bind(&CmdVelArbitratorNode::localCostmapCallback, this, _1)
        );

        /* SERVICE CLIENTS */
        // NAV2(SAFETY) state client
        behavior_server_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/behavior_server/get_state");
        bt_navigator_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/bt_navigator/get_state");
        controller_server_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/controller_server/get_state");
        global_costmap_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/global_costmap/global_costmap/get_state");
        local_costmap_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/local_costmap/local_costmap/get_state");
        map_server_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/map_server/get_state");
        planner_server_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/planner_server/get_state");
        smoother_server_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/smoother_server/get_state");
        velocity_smoother_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/velocity_smoother/get_state");
        waypoint_follower_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>("/waypoint_follower/get_state");

        /* TIMERS */
        // state machine timer callback (ref: CONTROL LOOP PERIOD)
        statemachine_timer_ = this->create_wall_timer(std::chrono::milliseconds(CONTROL_LOOP_PERIOD),
                                                      std::bind(&CmdVelArbitratorNode::statemachine_callback, this));

        // state client timer callback (ref: HEARTBEAT TIMEOUT)
        state_client_timer_ = this->create_wall_timer(std::chrono::milliseconds(HEARTBEAT_TIMEOUT),
                                                      std::bind(&CmdVelArbitratorNode::check_all_nav2_services, this));

        /* INITIALIZE VARS */         
        target_.header.frame_id = FRAME_ID;
        target_.pose.position.x = POSITION_X;
        target_.pose.position.y = POSITION_Y;
        target_.pose.position.z = POSITION_Z;
        target_.pose.orientation.x = ORI_X;
        target_.pose.orientation.y = ORI_Y;
        target_.pose.orientation.z = ORI_Z;
        // target_.pose.orientation.w = ORI_W;

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

private:
    StateMachine statemachine;
    sml::sm<StateMachine::sm_arbitrator> sm_;
    // main topics
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr             cmd_pub_;    // output: /cmd_vel
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          model_sub_;    // input: /cmd_vel_model
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          safety_sub_;    // input: /cmd_vel_safe
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr          manual_sub_;    // input: /cmd_vel_manual
    // other topics
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr       pose_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                safety_alive_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                safety_risk_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                model_heartbeat_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                safety_heartbeat_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                manual_heartbeat_sub_;
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr  safety_goal_status_sub_;
    rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr            local_costmap_sub_;
    // service client
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr behavior_server_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr bt_navigator_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr controller_server_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr global_costmap_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr local_costmap_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr map_server_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr planner_server_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr smoother_server_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr velocity_smoother_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr waypoint_follower_state_client_;
    // var
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr statemachine_timer_;
    rclcpp::TimerBase::SharedPtr state_client_timer_;
    rclcpp::Time last_model_time_;
    bool last_model_valid_ = false;
    rclcpp::Time last_safety_time_;
    bool last_safety_valid_ = false;
    rclcpp::Time last_manual_time_;
    bool last_manual_valid_ = false;
    rclcpp::Time last_model_alive_time_;
    rclcpp::Time last_safety_alive_time_;
    rclcpp::Time last_manual_alive_time_;
    rclcpp::Time rescue_start_time_;
    rclcpp::Time last_local_cost_map_time_;
    geometry_msgs::msg::Twist latest_model_;
    geometry_msgs::msg::Twist latest_safety_;
    geometry_msgs::msg::Twist latest_manual_;
    uint8_t latest_safety_status_;
    geometry_msgs::msg::PoseStamped target_;    // Initialized target pose
    nav2_msgs::msg::Costmap latest_local_costmap_;
    bool model_alive = true;
    bool safety_alive = true;
    bool manual_alive = true;
    bool safety_risk = false;
    bool rescue_started = false;
    bool return_to_ori_started  = false;
    bool skip = false;
    std::unordered_map<std::string, bool> nav2_service_ok_map_;
    size_t nav2_expected_service_count_ = 10;

    int count_test = 0;
    


    /* MESSAGE CALLBACK FUNCTIONS */
    void model_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_model_valid_ = true;
        latest_model_ = *msg;
        last_model_time_ = this->now();
        this->model_alive = true;
        statemachine.set_model_alive(this->model_alive);
        sm_.process_event(StateMachine::model_cmd_active{});
    }

    void safety_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_safety_valid_ = true;
        latest_safety_ = *msg;
        last_safety_time_ = this->now();
    }

    void manual_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_manual_valid_ = true;
        latest_manual_ = *msg;
        last_manual_time_ = this->now();
        this->return_to_ori_started = false;
        this->manual_alive = true;
        statemachine.set_manual_alive(this->manual_alive);
        sm_.process_event(StateMachine::manual_cmd_received{});
    }

    #if MODEL_HB_ENABLE
    void model_alive_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        rclcpp::Duration time_since_last_heartbeat = this->now() - last_model_alive_time_;
        rclcpp::Duration heartbeat_timeout = rclcpp::Duration::from_seconds(HEARTBEAT_TIMEOUT/1000.0);
        this->model_alive = time_since_last_heartbeat < heartbeat_timeout;
        last_model_alive_time_ = this->now();
        statemachine.set_model_alive(this->model_alive);
        }
    #endif

    #if SAFE_HB_ENABLE
    void safety_alive_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        rclcpp::Duration time_since_last_heartbeat = this->now() - last_safety_alive_time_;
        rclcpp::Duration heartbeat_timeout = rclcpp::Duration::from_seconds(HEARTBEAT_TIMEOUT/1000.0);
        this->safety_alive = time_since_last_heartbeat < heartbeat_timeout;
        last_safety_alive_time_ = this->now();
        statemachine.set_safety_alive(this->safety_alive);
        RCLCPP_ERROR(this->get_logger(), "SAFETY_NODE NOT ONLINE, SAFETY ALIVE: %d", this->safety_alive);
        sm_.process_event(StateMachine::safety_node_not_online{});
        }
    #endif

    #if MANUAL_HB_ENABLE
    void manual_alive_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        rclcpp::Duration time_since_last_heartbeat = this->now() - last_manual_alive_time_;
        rclcpp::Duration heartbeat_timeout = rclcpp::Duration::from_seconds(HEARTBEAT_TIMEOUT/1000.0);
        this->manual_alive = time_since_last_heartbeat < heartbeat_timeout;
        last_manual_alive_time_ = this->now();
        statemachine.set_manual_alive(this->manual_alive);
        }
    #endif

    #if EXT_SAFETY_ENABLE
    void safety_risk_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        safety_risk = true;
        statemachine.set_safety_risk(safety_risk);
        }
    #endif

    // check safety_controller status
    void safety_status_callback(const action_msgs::msg::GoalStatusArray::SharedPtr msg) {
        const auto &last_status = msg->status_list.back();  // 取最后一个
        uint8_t status_code = last_status.status;
        RCLCPP_INFO(this->get_logger(), "test status code*******************************: %d", status_code);
        bool activation = latest_safety_status_ != status_code;
        latest_safety_status_ = status_code;
        if ((statemachine.get_status() == 3 || statemachine.get_status() == 4) && activation) {
            if (!msg->status_list.empty())
            {
                // #if DEBUG_MODE
                // RCLCPP_INFO(this->get_logger(), "SAFETY_CONTROLLER STATUS CODE: %d", status_code);
                // #endif
                if (status_code == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED) {
                    RCLCPP_INFO(this->get_logger(), "SAFETY_CONTROLLER TARGET REACHED");
                    rescue_started = false;
                    this->return_to_ori_started = false;
                    sm_.process_event(StateMachine::destination_reached{});
                }
                else if (status_code == action_msgs::msg::GoalStatus::STATUS_ABORTED) {
                    if (rescue_started) {
                        RCLCPP_ERROR(this->get_logger(), "RESCUE FAILED, SAFETY_CONTROLLER MISSION ABORTED");
                        rescue_started = false;
                        this->return_to_ori_started = false;
                        sm_.process_event(StateMachine::rescue_failed{});
                    } else {
                        RCLCPP_WARN(this->get_logger(), "SAFETY_CONTROLLER MISSION ABORTED, NEED RESCUE");
                        rescue_start_time_ = this->now();
                        rescue_started = true;
                        // this->return_to_ori_started = false;
                        sm_.process_event(StateMachine::vehicle_not_moving{});
                    }
                }
                else if (status_code == action_msgs::msg::GoalStatus::STATUS_CANCELED) {
                    RCLCPP_INFO(this->get_logger(), "SAFETY_CONTROLLER MISSION CANCELED");
                    rescue_started = false;
                    this->return_to_ori_started = false;
                    sm_.process_event(StateMachine::destination_reached{});
                }
            }
        }
    }

    // receive local costmap
    void localCostmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg) {
        latest_local_costmap_ = *msg;
        last_local_cost_map_time_ = this->now();
        int cost = getRobotCostInCostmap(latest_local_costmap_, tf_buffer_, this->get_logger());
        // #if DEBUG_MODE
        // RCLCPP_INFO(this->get_logger(), "Robot is currently at costmap value: %d", cost);
        // #endif
        if (cost >= SAFETY_THRESHOLD) {
            sm_.process_event(StateMachine::safety_risk_detected{});
            this->safety_risk = true;
            statemachine.set_safety_risk(this->safety_risk);
        } else {
            this->safety_risk = false;
            statemachine.set_safety_risk(this->safety_risk);
        }
    }

    /* TIMER CALLBACK FUNCTIONS */
    void check_all_nav2_services()
    {
        nav2_service_ok_map_.clear();

        auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

        send_request(behavior_server_state_client_, request, "Behavior Server");
        send_request(bt_navigator_state_client_, request, "BT Navigator");
        send_request(controller_server_state_client_, request, "Controller Server");
        send_request(global_costmap_state_client_, request, "Global Costmap");
        send_request(local_costmap_state_client_, request, "Local Costmap");
        send_request(map_server_state_client_, request, "Map Server");
        send_request(planner_server_state_client_, request, "Planner Server");
        send_request(smoother_server_state_client_, request, "Smoother Server");
        send_request(velocity_smoother_state_client_, request, "Velocity Smoother");
        send_request(waypoint_follower_state_client_, request, "Waypoint Follower");
    }

    void send_request(
            rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client,
            std::shared_ptr<lifecycle_msgs::srv::GetState::Request> request,
            const std::string &server_name
    )
    {
        using ServiceResponseFuture = rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture;
        const std::chrono::milliseconds timeout_duration(SERVICE_CLIENT_TIMEOUT);
        auto timer_actived = std::make_shared<bool>(false);
        auto timeout_timer = this->create_wall_timer(
                timeout_duration, [this, server_name, timer_actived]() {
                    if (!(*timer_actived)) {
                        *timer_actived = true;
                        RCLCPP_ERROR(this->get_logger(), "%s request timed out. SAFETY_NODE is offline.", server_name.c_str());
                        // RESPONSE TIMEOUT HANDLER
                        this->safety_alive = false; // SAFETY NODE IS OFFLINE
                        statemachine.set_safety_alive(safety_alive);
                        sm_.process_event(StateMachine::safety_node_not_online{});
                    }
                });
        auto response_received_callback = [this, server_name, timeout_timer](ServiceResponseFuture future) {
            try {
                timeout_timer->cancel();
                auto response = future.get();
                nav2_service_ok_map_[server_name] = true;
                // #if DEBUG_MODE
                // RCLCPP_INFO(this->get_logger(), "%s state: %s", server_name.c_str(), response->current_state.label.c_str());
                // #endif
                if (nav2_service_ok_map_.size() == nav2_expected_service_count_) {
                    bool all_ok = std::all_of(nav2_service_ok_map_.begin(), nav2_service_ok_map_.end(),
                    [](const auto &pair) { return pair.second; });
                    if (all_ok && !safety_alive) {
                        safety_alive = true;
                        statemachine.set_safety_alive(true);
                        sm_.process_event(StateMachine::safety_node_revived{});
                        RCLCPP_INFO(this->get_logger(), "SAFETY_NODE revived: all services responded.");
                    }
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Error when getting %s state: %s", server_name.c_str(), e.what());
            }
        };
        // asynchronous request
        client->async_send_request(request, response_received_callback);
    }

    int getRobotCostInCostmap(
    const nav2_msgs::msg::Costmap &costmap,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Logger logger) {
        try {
            // 查找 base_link 相对于 costmap 坐标系的变换
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer->lookupTransform(
                    costmap.header.frame_id,
                    "base_link",
                    tf2::TimePointZero,
                    tf2::durationFromSec(0.2));

            double x = tf.transform.translation.x;
            double y = tf.transform.translation.y;

            double origin_x = costmap.metadata.origin.position.x;
            double origin_y = costmap.metadata.origin.position.y;
            double resolution = costmap.metadata.resolution;
            int width = costmap.metadata.size_x;
            int height = costmap.metadata.size_y;

            int i = static_cast<int>((x - origin_x) / resolution);
            int j = static_cast<int>((y - origin_y) / resolution);

            if (i >= 0 && i < width && j >= 0 && j < height) {
                int index = j * width + i;
                return static_cast<int>(costmap.data[index]);
            } else {
                RCLCPP_WARN(logger, "Calculated index (%d, %d) is out of bounds", i, j);
            }
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(logger, "TF lookup failed: %s", ex.what());
        }

        return -1; // 表示错误或不可获取
    }


    void statemachine_callback() {
        #if DEBUG_MODE
        RCLCPP_INFO(this->get_logger(), "Current status: %d", statemachine.get_status());
        #endif
        // check safety risk
        if (this->safety_risk) {
            if (!(this->return_to_ori_started)) {
                this->return_to_ori_started = true;
                target_.header.stamp = this->get_clock()->now();
                pose_pub_->publish(target_);
                RCLCPP_ERROR(this->get_logger(), "Published new goal due to safety risk");
            }
        }
        // Check model cmd timeout
        if (statemachine.get_status() == 1) {
            rclcpp::Duration time_since_last_model_cmd = this->now() - last_model_time_;
            if (time_since_last_model_cmd > rclcpp::Duration::from_seconds(MODEL_CMD_TIMEOUT / 1000.0)) {
                RCLCPP_WARN(this->get_logger(), "Model cmd_vel timeout");
                sm_.process_event(StateMachine::model_cmd_timeout{});
            }
        }
        // Check manual cmd timeout
        if (statemachine.get_status() == 5) {
            rclcpp::Duration time_since_last_manual_cmd = this->now() - last_manual_time_;
            if (time_since_last_manual_cmd > rclcpp::Duration::from_seconds(MANUAL_CMD_TIMEOUT / 1000.0)) {
                RCLCPP_WARN(this->get_logger(), "Manual node terminated");
                sm_.process_event(StateMachine::manual_cmd_terminated{});
            }
        }
        // Check model cmd finished or model node died
        if (statemachine.get_status() == 2) {
            rclcpp::Duration time_since_last_model_alive = this->now() - last_model_time_;
            if (time_since_last_model_alive < rclcpp::Duration::from_seconds(MODEL_CMD_TIMEOUT / 1000.0)) {
                RCLCPP_WARN(this->get_logger(), "Model cmd_vel on time");
                sm_.process_event(StateMachine::model_cmd_resumed{});
            }
            if (time_since_last_model_alive > rclcpp::Duration::from_seconds(MODEL_NODE_DEACTIVATION / 1000.0)) {
                RCLCPP_WARN(this->get_logger(), "Model node deactivated due to timeout");
                this->model_alive = false;
                statemachine.set_model_alive(model_alive);
                this->return_to_ori_started = true;
                target_.header.stamp = this->get_clock()->now();
                pose_pub_->publish(target_);
                sm_.process_event(StateMachine::model_node_not_online{});
            }
        }
        // Publish cmd_vel based on the current state
        geometry_msgs::msg::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.linear.y = 0.0;
        cmd_vel_msg.linear.z = 0.0;
        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = 0.0;
        switch (statemachine.get_status()) {
            case 0: // IDLE
                break;
            case 1: // MODEL_OK
                if (last_model_valid_) {
                    cmd_vel_msg = latest_model_;
                    last_model_valid_ = false;
                }
                break;
            case 2: // MODEL_TIMEOUT
                cmd_vel_msg = latest_model_;
                break;
            case 3: // SAFETY_OVERRIDE
                if (last_safety_valid_) {
                    cmd_vel_msg = latest_safety_;
                    last_safety_valid_ = false;
                }
                break;
            case 4: // SAFETY_FAILURE
                {
                    rclcpp::Duration time_since_rescue_started = this->now() - rescue_start_time_;
                    if (time_since_rescue_started < rclcpp::Duration::from_seconds(RESCUE_DURATION / 1000.0)) {
                        cmd_vel_msg.linear.x = -1 * RESCUE_SPEED; // Reverse slowly   
                        cmd_vel_msg.angular.z = 0.0;
                    } else {
                        this->return_to_ori_started = true;
                        target_.header.stamp = this->get_clock()->now();
                        pose_pub_->publish(target_);
                        sm_.process_event(StateMachine::rescue_completed{});
                    }
                }
                break;
            case 5: // MANUAL_OVERRIDE
                if (last_manual_valid_) {
                    cmd_vel_msg = latest_manual_;
                    last_manual_valid_ = false;
                }
                break;
            case 6: // STOP
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown status: %d", statemachine.get_status());
                return; // Exit if status is unknown
        }
        cmd_pub_->publish(cmd_vel_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelArbitratorNode>());
    rclcpp::shutdown();
    return 0;
}
