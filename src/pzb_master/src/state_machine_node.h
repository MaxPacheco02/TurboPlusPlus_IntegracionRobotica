#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <queue>
#include <iostream>
#include <vector>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_srvs/srv/empty.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/utils.h"

#include "pzb_msgs/msg/lift_state.hpp"
#include "pzb_msgs/srv/lift_service.hpp"
#include "pzb_msgs/srv/movement_request_service.hpp"
#include "pzb_msgs/msg/trailer_bearing.hpp"

using namespace std::chrono_literals;

struct Goal
{
    double x, y, z;
};

class StateMachineNode : public rclcpp::Node
{
public:
    StateMachineNode();

private:
    enum class State
    {
        PALLET_SEARCH,
        PALLET_COLLECTION,
        TRAILER_SEARCH,
        PALLET_DELIVERY
    };

    enum class LIFTSTATE
    {
        IDLE,
        PRELOAD,
        LOAD,
        RAISE,
        UNLOAD
    };

    enum class PSState
    {
        IDLE,
        NAVIGATE,
    };

    enum class PCState
    {
        IDLE,
        PRELOAD,
        ALIGN,
        APPROACH,
        LOAD
    };

    enum class TSState
    {
        LOAD,
        SEARCH,
    };

    enum class PDState
    {
        LOAD,
        RAISE,
        ALIGN,
        APPROACH,
        UNLOAD,
        BACKUP,
        IDLE
    };

    // State variables
    State state_;
    LIFTSTATE lift_state_;
    PSState ps_state_;
    PCState pc_state_;
    TSState ts_state_;
    PDState pd_state_;

    // State completion flags
    bool pallet_search_completed_;
    bool pallet_collection_completed_;
    bool trailer_search_completed_;
    bool pallet_delivery_completed_;

    // Operation flags
    bool preload_sent_;
    bool ref_reached_;
    bool approach_sent_;
    bool raise_sent_;
    bool approach_reached_;
    bool pallet_aligned_;
    bool loaded_lift_;
    bool unload_sent_;
    bool backup_sent_;
    bool idle_sent_;
    bool req_sent_;
    bool pallet_detected_;
    bool ar_pallet_detected_;
    bool qr_detected_;
    bool trailer_detected_;

    int req_id_;
    int reached_id_;

    // Distance
    double pallet_distance_;
    double trailer_distance_;
    std::string qr_on_watch_;
    std::string trailer_goal_name_;

    std::vector<Goal> v_goals;

    rclcpp::Time last_ar_pallet_update_,
        last_qr_update_;

    // ROS Interfaces
    rclcpp::Subscription<pzb_msgs::msg::LiftState>::SharedPtr lift_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr qr_on_watch_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr tracker_ref_id_sub_;
    rclcpp::Subscription<pzb_msgs::msg::TrailerBearing>::SharedPtr trailer_detection_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    rclcpp::Client<pzb_msgs::srv::LiftService>::SharedPtr lift_client_;
    rclcpp::Client<pzb_msgs::srv::MovementRequestService>::SharedPtr movement_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr rst_ref_id_client_;

    rclcpp::TimerBase::SharedPtr update_timer_;

    visualization_msgs::msg::MarkerArray marker_arr;

    void update();
    void state_switch();
    std::string state_to_string(State s);
    void pallet_search();
    void pallet_collection();
    void trailer_search();
    void pallet_delivery();
    void set_v_goals();
    Goal get_next_rand_wp();

    void add_marker(Goal goal, int id, int color);
    void call_lift_service(int8_t desired_lift_state);
    void call_movement_service(std::string type, Goal goal, bool is_alignment_to_trailer, std::string trailer_choice, double approach_distance);
    void call_rst_ref_id_srv();

    int color_list[6][4]{
        {1, 0, 0, 1},
        {0, 1, 0, 1},
        {0, 0, 1, 1},
        {1, 1, 0, 1},
        {0, 0, 0, 1},
        {0, 0, 0, 0}};

    std::map<std::string, double> cam_bearing_angles = {
        {"diagonal", NAN},
        {"oxidado", NAN},
        {"rojo", NAN},
    };
};