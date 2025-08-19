#include "state_machine_node.h"

StateMachineNode::StateMachineNode() : Node("state_machine_node")
{
    using std::placeholders::_1;

    set_v_goals();

    lift_sub_ = this->create_subscription<pzb_msgs::msg::LiftState>(
        "/current_lift_state", 10,
        [this](const pzb_msgs::msg::LiftState &msg)
        {
            loaded_lift_ = ((msg.state == pzb_msgs::msg::LiftState::LOAD) ||
                            (msg.state == pzb_msgs::msg::LiftState::RAISE));
            lift_state_ = msg.state;
        });

    pallet_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/pallet/goal_pose", 10,
        [this](const geometry_msgs::msg::PoseStamped &msg)
        {
            last_ar_pallet_update_ = this->get_clock()->now();
        });

    qr_on_watch_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/qr_code_data", 10,
        [this](const std_msgs::msg::String &msg)
        {
            qr_on_watch_ = msg.data;
            last_qr_update_ = this->get_clock()->now();
        });

    tracker_ref_id_sub_ = this->create_subscription<std_msgs::msg::Int64>(
        "/movement_ref_id", 10,
        [this](const std_msgs::msg::Int64 &msg)
        {
            reached_id_ = msg.data;
        });

    trailer_detection_sub_ = this->create_subscription<pzb_msgs::msg::TrailerBearing>(
        "/yolo_bearing_angles", 10,
        [this](const pzb_msgs::msg::TrailerBearing &msg)
        {
            cam_bearing_angles["diagonal1"] = msg.diagonal1;
            cam_bearing_angles["oxidado1"] = msg.oxidado1;
            cam_bearing_angles["rojo1"] = msg.rojo1;
            cam_bearing_angles["diagonal2"] = msg.diagonal2;
            cam_bearing_angles["oxidado2"] = msg.oxidado2;
            cam_bearing_angles["rojo2"] = msg.rojo2;
        });

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/virtual_goals_marker_array", 10);

    lift_client_ = this->create_client<pzb_msgs::srv::SetLift>("/set_lift_state");
    movement_client_ = this->create_client<pzb_msgs::srv::MovementRequestService>("movement_service");
    rst_ref_id_client_ = this->create_client<std_srvs::srv::Empty>("reset_ref_id_service");

    update_timer_ = this->create_wall_timer(10ms, std::bind(&StateMachineNode::update, this));

    // Initialize state variables
    state_ = State::PALLET_SEARCH;
    lift_state_ = 0;
    ps_state_ = PSState::IDLE;
    ts_state_ = TSState::LOAD;
    pd_state_ = PDState::LOAD;

    pallet_search_completed_ = false;
    pallet_collection_completed_ = false;
    trailer_search_completed_ = false;
    pallet_delivery_completed_ = false;

    // Initialize flags
    loaded_lift_ = false;
    trailer_detected_ = false;
    unload_sent_ = false;
    approach_sent_ = false;
    ref_reached_ = false;
    pallet_aligned_ = false;
    approach_reached_ = false;
    preload_sent_ = false;
    raise_sent_ = false;
    pallet_detected_ = false;
    backup_sent_ = false;
    idle_sent_ = false;
    req_sent_ = false;

    req_id_ = 0;
    reached_id_ = 0;
    trailer_distance_ = 0.30; // Distance to trailer in meters
    pallet_distance_ = 0.20;  // Distance to pallet in meters

    // Add 10 seconds to latest update to avoid false positives in detection condition
    last_ar_pallet_update_ = this->get_clock()->now() - rclcpp::Duration(0, 10000 * 1e6);
    last_qr_update_ = this->get_clock()->now() - rclcpp::Duration(0, 10000 * 1e6);
}

void StateMachineNode::update()
{
    rclcpp::Time now = this->get_clock()->now();

    if (trailer_goal_name_ != "")
    {
        RCLCPP_INFO(this->get_logger(), "trailer name: %s, Bearing desired: %f, is ok? %d",
                    trailer_goal_name_.c_str(),
                    cam_bearing_angles[trailer_goal_name_ + "1"],
                    std::isnan(cam_bearing_angles[trailer_goal_name_ + "1"]));
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "QR NOT DETECTED!");
    }

    // 100 ms
    if (now - last_ar_pallet_update_ < rclcpp::Duration(0, 100 * 1e6))
        ar_pallet_detected_ = true;
    else
        ar_pallet_detected_ = false;

    if (now - last_qr_update_ < rclcpp::Duration(0, 100 * 1e6))
        qr_detected_ = true;
    else
        qr_detected_ = false;

    // Ideally, it should be like this. But the qr detection is not optimized right now.
    // pallet_detected_ = ar_pallet_detected_ && qr_detected_;
    pallet_detected_ = ar_pallet_detected_;

    // RCLCPP_INFO(this->get_logger(), "ar: %d, qr: %d, pallet: %d", ar_pallet_detected_, qr_detected_, pallet_detected_);

    state_switch();
}

void StateMachineNode::state_switch()
{
    RCLCPP_INFO(this->get_logger(), "Main state: %s", state_to_string(state_).c_str());

    switch (state_)
    {
    case State::PALLET_SEARCH:
        if (pallet_search_completed_)
        {
            pallet_search_completed_ = false;
            ps_state_ = PSState::IDLE;
            pc_state_ = PCState::IDLE;
            state_ = State::PALLET_COLLECTION;
        }
        else
        {
            pallet_search();
        }
        break;

    case State::PALLET_COLLECTION:
        if (pallet_collection_completed_)
        {
            pallet_collection_completed_ = false;
            pc_state_ = PCState::IDLE;
            ts_state_ = TSState::LOAD;
            // state_ = State::TRAILER_SEARCH;
            // TODO: FIX THIS trailer search JE!
            state_ = State::PALLET_DELIVERY;
        }
        else
        {
            pallet_collection();
        }
        break;

    case State::TRAILER_SEARCH:
        if (trailer_search_completed_)
        {
            trailer_search_completed_ = false;
            ts_state_ = TSState::LOAD;
            pd_state_ = PDState::LOAD;
            state_ = State::PALLET_DELIVERY;
        }
        else
        {
            trailer_search();
        }
        break;

    case State::PALLET_DELIVERY:
        if (pallet_delivery_completed_)
        {
            pallet_delivery_completed_ = false;
            pd_state_ = PDState::LOAD;
            pc_state_ = PCState::IDLE;
            state_ = State::PALLET_SEARCH;
        }
        else
        {
            pallet_delivery();
        }
        break;
    }
}

std::string StateMachineNode::state_to_string(State s)
{
    switch (s)
    {
    case State::PALLET_SEARCH:
        return "PALLET_SEARCH";
    case State::PALLET_COLLECTION:
        return "PALLET_COLLECTION";
    case State::TRAILER_SEARCH:
        return "TRAILER_SEARCH";
    case State::PALLET_DELIVERY:
        return "PALLET_DELIVERY";
    default:
        return "UNKNOWN";
    }
}

Goal StateMachineNode::get_next_rand_wp()
{
    if (v_goals.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "VIRTUAL GOALS NOT SET");
        return Goal{};
    }
    return v_goals[std::rand() % v_goals.size()];
}

void StateMachineNode::pallet_search()
{
    switch (ps_state_)
    {
    case PSState::IDLE:
        RCLCPP_INFO(this->get_logger(), "[PS] IDLE - Starting pallet search");
        ps_state_ = PSState::NAVIGATE;
        // req_id_++;
        // call_movement_service("nav", get_next_rand_wp(), false, "", 0);

    case PSState::NAVIGATE:
        RCLCPP_INFO(this->get_logger(), "[PS] NAVIGATE - Scanning for pallets...");
        if (pallet_detected_ && !loaded_lift_)
        {
            trailer_goal_name_ = qr_on_watch_;
            req_id_ = 0;
            call_rst_ref_id_srv();
            pallet_search_completed_ = true;
            RCLCPP_INFO(this->get_logger(), "[PS] Pallet detected - search complete");
        }
        else if (req_id_ == reached_id_)
        {
            ps_state_ = PSState::IDLE;
        }
        break;
    }
}

void StateMachineNode::pallet_collection()
{
    switch (pc_state_)
    {
    case PCState::IDLE:
        RCLCPP_INFO(this->get_logger(), "[PC] IDLE");
        if (!req_sent_)
        {
            call_movement_service("align", Goal{}, false, "", 0);
            req_id_++;
            req_sent_ = true;
        }
        if (req_sent_ && (req_id_ == reached_id_))
        {
            pc_state_ = PCState::PRELOAD;
            req_sent_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "req sent: %d, req_id: %d, reached_id: %d", req_sent_, req_id_, reached_id_);
        break;

    case PCState::PRELOAD:
        RCLCPP_INFO(this->get_logger(), "[PC] PRELOAD");
        if (!preload_sent_)
        {
            call_lift_service(pzb_msgs::msg::LiftState::PRELOAD);
            preload_sent_ = true;
        }
        if (lift_state_ == pzb_msgs::msg::LiftState::PRELOAD)
        {
            pc_state_ = PCState::ALIGN;
            preload_sent_ = false;
        }
        // RCLCPP_INFO(this->get_logger(), "lift state: %d", lift_state_);


        break;

    case PCState::ALIGN:
        RCLCPP_INFO(this->get_logger(), "[PC] ALIGN");
        if (!approach_sent_)
        {
            call_movement_service("approach", Goal{}, false, "", pallet_distance_);
            req_id_++;
            approach_sent_ = true;
        }
        if (approach_sent_ && (req_id_ == reached_id_))
        {
            pc_state_ = PCState::APPROACH;
            approach_sent_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "req sent: %d, req_id: %d, reached_id: %d", approach_sent_, req_id_, reached_id_);

        break;

    case PCState::APPROACH:
        RCLCPP_INFO(this->get_logger(), "[PC] APPROACH");
        if (!raise_sent_)
        {
            call_lift_service(pzb_msgs::msg::LiftState::LOAD);
            raise_sent_ = true;
        }
        if (lift_state_ == pzb_msgs::msg::LiftState::LOAD)
        {
            pc_state_ = PCState::LOAD;
            raise_sent_ = false;
        }
        break;

    case PCState::LOAD:
        RCLCPP_INFO(this->get_logger(), "[PC] LOAD");
        if (loaded_lift_)
        {
            RCLCPP_INFO(this->get_logger(), "[PC] Pallet loaded successfully.");
            pallet_collection_completed_ = true;
        }
        break;
    }
}

void StateMachineNode::trailer_search()
{
    switch (ts_state_)
    {
    case TSState::LOAD:
        RCLCPP_INFO(this->get_logger(), "[TS] LOAD - Starting trailer search");
        ts_state_ = TSState::SEARCH;
        req_id_++;
        call_movement_service("nav", get_next_rand_wp(), false, "", 0);
        break;

    case TSState::SEARCH:
        RCLCPP_INFO(this->get_logger(), "[TS] SEARCH - Looking for trailer...");
        RCLCPP_INFO(this->get_logger(), "angle to look for: %d", cam_bearing_angles[trailer_goal_name_ + "1"]);
        RCLCPP_INFO(this->get_logger(), "trailer name: %s", trailer_goal_name_.c_str());
        // If the goal trailer is seen...
        if (!std::isnan(cam_bearing_angles[trailer_goal_name_ + "1"]) && trailer_goal_name_ != "" && loaded_lift_)
        {
            trailer_search_completed_ = true; // Signal completion
            req_id_ = 0;
            call_rst_ref_id_srv();
            RCLCPP_INFO(this->get_logger(), "[TS] Trailer detected - search complete");
        }
        else if (req_id_ == reached_id_)
        {
            ts_state_ = TSState::LOAD;
        }
        break;
    }
}

void StateMachineNode::pallet_delivery()
{
    switch (pd_state_)
    {
    case PDState::LOAD:
        RCLCPP_INFO(this->get_logger(), "[PD] LOADED");

        RCLCPP_INFO(this->get_logger(), "req sent: %d, req_id: %d, reached_id: %d", req_sent_, req_id_, reached_id_);

        if (!req_sent_)
        {
            call_movement_service("align", Goal{}, true, trailer_goal_name_, 0);
            req_id_++;
            req_sent_ = true;
        }
        if (req_sent_ && req_id_ == reached_id_)
        {
            pd_state_ = PDState::RAISE;
            req_sent_ = false;
        }
        break;

    case PDState::RAISE:
        RCLCPP_INFO(this->get_logger(), "[PD] RAISE");
        if (!raise_sent_)
        {
            call_lift_service(pzb_msgs::msg::LiftState::RAISE);
            raise_sent_ = true;
        }
        if (lift_state_ == pzb_msgs::msg::LiftState::RAISE)
        {
            pd_state_ = PDState::ALIGN;
            raise_sent_ = false;
        }

        break;

    case PDState::ALIGN:
        RCLCPP_INFO(this->get_logger(), "[PD] ALIGN");
        if (!approach_sent_)
        {
            call_movement_service("approach", Goal{}, false, "", trailer_distance_);
            req_id_++;
            approach_sent_ = true;
        }
        if (approach_sent_ && req_id_ == reached_id_)
        {
            pd_state_ = PDState::APPROACH;
            approach_sent_ = false;
        }
        break;

    case PDState::APPROACH:
        RCLCPP_INFO(this->get_logger(), "[PD] APPROACH");
        if (!unload_sent_)
        {
            call_lift_service(pzb_msgs::msg::LiftState::UNLOAD);
            unload_sent_ = true;
        }
        if (lift_state_ == pzb_msgs::msg::LiftState::UNLOAD)
        {
            pd_state_ = PDState::UNLOAD;
            unload_sent_ = false;
        }
        break;

    case PDState::UNLOAD:
        RCLCPP_INFO(this->get_logger(), "[PD] UNLOAD");

        if (!backup_sent_)
        {
            // backup movement "back-off?"
            call_movement_service("approach", Goal{}, false, "", -trailer_distance_);
            req_id_++;
            backup_sent_ = true;
        }
        if (backup_sent_ && req_id_ == reached_id_)
        {
            pd_state_ = PDState::BACKUP;
            backup_sent_ = false;
        }
        break;

    case PDState::BACKUP:
        RCLCPP_INFO(this->get_logger(), "[PD] BACKUP");
        if (!idle_sent_)
        {
            call_lift_service(pzb_msgs::msg::LiftState::IDLE);
            idle_sent_ = true;
        }
        if (lift_state_ == pzb_msgs::msg::LiftState::IDLE)
        {
            pd_state_ = PDState::IDLE;
            idle_sent_ = false;
        }

        break;

    case PDState::IDLE:
        RCLCPP_INFO(this->get_logger(), "[PD] IDLE → return to pallet search");
        state_ = State::PALLET_SEARCH;
        break;
    }
}

void StateMachineNode::set_v_goals()
{
    this->declare_parameter("v_goal_list", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> v_goal_list = this->get_parameter("v_goal_list").as_string_array();

    v_goals.clear();

    for (std::string &v_goal : v_goal_list)
    {
        this->declare_parameter(v_goal + ".x", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter(v_goal + ".y", rclcpp::PARAMETER_DOUBLE);
        this->declare_parameter(v_goal + ".z", rclcpp::PARAMETER_DOUBLE);

        double x_ = this->get_parameter(v_goal + ".x").as_double();
        double y_ = this->get_parameter(v_goal + ".y").as_double();
        double z_ = this->get_parameter(v_goal + ".z").as_double();

        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "GOAL %s. %f, %f", v_goal.c_str(), x_, y_);

        Goal new_goal{x_, y_, z_};
        v_goals.push_back(new_goal);
        add_marker(new_goal, v_goals.size(), 0);
    }
}

void StateMachineNode::call_lift_service(int8_t desired_lift_state)
{
    auto request = std::make_shared<pzb_msgs::srv::SetLift::Request>();
    request->desired_state = desired_lift_state;

    while (!lift_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Lift client -> Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lift client -> service not available, waiting again...");
    }

    auto result = lift_client_->async_send_request(request);
    return;
}

void StateMachineNode::call_rst_ref_id_srv()
{
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    while (!rst_ref_id_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "rst_ref_id_client_ -> Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "rst_ref_id_client_ -> service not available, waiting again...");
    }

    auto result = rst_ref_id_client_->async_send_request(request);
    return;
}

void StateMachineNode::call_movement_service(std::string type, Goal goal, bool is_alignment_to_trailer, std::string trailer_choice, double approach_distance)
{
    geometry_msgs::msg::PoseStamped ref_pose;
    ref_pose.pose.position.x = goal.x;
    ref_pose.pose.position.y = goal.y;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, goal.z);
    tf2::convert(quat, ref_pose.pose.orientation);

    auto request = std::make_shared<pzb_msgs::srv::MovementRequestService::Request>();
    request->type = type;
    request->ref_pose = ref_pose;
    request->is_alignment_to_trailer = is_alignment_to_trailer;
    request->trailer_choice = trailer_choice;
    request->approach_distance = approach_distance;

    while (!movement_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "movement_client_ -> Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "movement_client_ -> service not available, waiting again...");
    }

    auto result = movement_client_->async_send_request(request);
    return;
}

void StateMachineNode::add_marker(Goal goal, int id, int color)
{
    visualization_msgs::msg::Marker marker;

    int r{color_list[color][0]},
        g{color_list[color][1]},
        b{color_list[color][2]},
        a{color_list[color][3]};

    marker.header.frame_id = "world";
    marker.color = std_msgs::build<std_msgs::msg::ColorRGBA>().r(r).g(g).b(b).a(a);
    marker.action = 0;
    marker.id = id;
    marker.type = 3;
    marker.scale = geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.1).y(0.1).z(0.1);
    marker.pose.position.x = goal.x;
    marker.pose.position.y = goal.y;
    marker.pose.position.z = 0.25;
    marker_arr.markers.push_back(marker);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());
    rclcpp::shutdown();
    return 0;
}
