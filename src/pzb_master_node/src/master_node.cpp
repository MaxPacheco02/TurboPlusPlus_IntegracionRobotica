#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <queue>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/utils.h"

#include "pzb_msgs/msg/lift_state.hpp"
#include "pzb_msgs/srv/lift_service.hpp"

using namespace std::chrono_literals;

class MasterNode : public rclcpp::Node
{
public:
    MasterNode() : Node("state_machine")
    {
        using std::placeholders::_1;

        lift_sub_ = this->create_subscription<pzb_msgs::msg::LiftState>(
            "/lift_state", 10,
            [this](const pzb_msgs::msg::LiftState &msg)
            {
                loaded_lift = (msg.state == pzb_msgs::msg::LiftState::LIFT_LOADED);
            });

        pallet_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pzb/pallet_pos", 10,
            [this](const geometry_msgs::msg::PoseStamped &msg)
            {
                last_pallet_update_ = this->get_clock()->now();
                pallet_detected = true;
            });

        lift_client_ = this->create_client<pzb_msgs::srv::LiftService>("lift_service");

        update_timer_ = this->create_wall_timer(10ms, std::bind(&MasterNode::state_switch, this));

        // Initialize state variables
        state_ = State::PALLET_SEARCH;
        lift_state_ = LIFTSTATE::IDLE;
        ps_state = PSState::IDLE;
        pc_state_ = PCState::IDLE;
        ts_state_ = TSState::LOAD;
        pd_state_ = PDState::LOADED;

        // Initialize flags
        loaded_lift_ = false;
        pallet_detected_ = false;
        trailer_detected_ = false;
        unload_sent_ = false;
        approach_sent_ = false;
        ref_reached_ = false;
        pallet_aligned_ = false;
        approach_reached_ = false;
        preload_sent_ = false;
        raise_sent_ = false;
        pallet_detected_ = false;
        trailer_detected_ = false;
        backup_sent_ = false;

        trailer_distance_ = 0.20; // Distance to trailer in meters
        pallet_distance_ = 0.05; // Distance to pallet in meters
        last_pallet_update_ = this->get_clock()->now();
    }

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
        LOADED,
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
    PCState pc_state_;
    PDState pd_state_;

    bool preload_sent_;
    bool ref_reached_;
    bool approach_sent_;
    bool raise_sent_;
    bool approach_reached_;
    bool pallet_aligned_;
    bool loaded_lift_;
    bool unload_sent_;
    bool backup_sent_;
    bool pallet_detected_;
    bool trailer_detected_;

    // Distance 
    double pallet_distance_;
    double trailer_distance_;

    rclcpp::Time last_pallet_update_;

    // ROS Interfaces
    rclcpp::Subscription<pzb_msgs::msg::LiftState>::SharedPtr lift_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_sub_;
    rclcpp::Client<pzb_msgs::srv::LiftService>::SharedPtr lift_client_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    void state_switch()
    {
        RCLCPP_INFO(this->get_logger(), "Main state: %s", state_to_string(state).c_str());

        switch (state_)
        {
        case State::PALLET_SEARCH:
            if (pallet_detected_ && !loaded_lift_)
            {
                pc_state_ = PCState::IDLE;
                state_ = State::PALLET_COLLECTION;
            }
            else
            {
                pallet_search();
            }
            break;

        case State::PALLET_COLLECTION:
            if (loaded_lift_)
            {
                ts_state_ = TSState::LOAD;
                state_ = State::TRAILER_SEARCH;
            }
            else
            {
                pallet_collection();
            }
            break;

        case State::TRAILER_SEARCH:
            if (trailer_detected_ && loaded_lift_)
            {
                pd_state_ = PDState::LOADED;
                state_ = State::PALLET_DELIVERY;
            }
            else
            {
                trailer_search();
            }
            break;

        case State::PALLET_DELIVERY:
            if (!loaded_lift_)
            {
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

    std::string state_to_string(State s)
    {
        switch (s)
        {
        case State::PALLET_SEARCH: return "PALLET_SEARCH";
        case State::PALLET_COLLECTION: return "PALLET_COLLECTION";
        case State::TRAILER_SEARCH: return "TRAILER_SEARCH";
        case State::PALLET_DELIVERY: return "PALLET_DELIVERY";
        default: return "UNKNOWN";
        }
    }

    void pallet_search()
    {
        RCLCPP_INFO(this->get_logger(), "[PALLET_SEARCH] Scanning for pallets...");
    }

    void pallet_collection()
    {
        switch (pc_state_)
        {
        case PCState::IDLE:
            RCLCPP_INFO(this->get_logger(), "[PC] IDLE");
            if (!preload_sent_) {
                call_lift_service(LIFTSTATE::PRELOAD); 
                preload_sent_ = true;
            }
            if (lift_state_ == LIFTSTATE::PRELOAD) {
                pc_state_ = PCState::PRELOAD;
                preload_sent_ = false;
            }
            break;

        case PCState::PRELOAD:
            RCLCPP_INFO(this->get_logger(), "[PC] PRELOAD");
            if (!ref_reached_) {
                call_unreached_ref();
                call_align_pallet_service();
                ref_reached_ = true;
            }
            if (reference_reached() && pallet_aligned()) {
                pc_state_ = PCState::ALIGN;
                ref_reached_ = false;
            }
            break;

        case PCState::ALIGN:
            RCLCPP_INFO(this->get_logger(), "[PC] ALIGN");
            if (!approach_sent_) {
                call_approach_service(pallet_distance_);
                approach_sent_ = true;
            }
            if (approach_reached()) {
                pc_state_ = PCState::APPROACH;
                approach_sent_ = false;
            }
            break;

        case PCState::APPROACH:
            RCLCPP_INFO(this->get_logger(), "[PC] APPROACH");
            if (!raise_sent_) {
                call_lift_service(LIFTSTATE::LOAD);
                raise_sent_ = true;
            }
            if (lift_state_ == LIFTSTATE::LOAD) {
                pc_state_ = PCState::LOAD;
                raise_sent_ = false;
                loaded_lift_ = true;
            }
            break;

        case PCState::LOAD:
            RCLCPP_INFO(this->get_logger(), "[PC] LOAD");
            if (loaded_lift_)
            {
                RCLCPP_INFO(this->get_logger(), "[PC] Pallet loaded successfully.");
                state_ = State::TRAILER_SEARCH;
            }
            break;
        }
    }

    void trailer_search()
    {
        RCLCPP_INFO(this->get_logger(), "[TRAILER_SEARCH] Looking for trailer...");
    }

    void pallet_delivery()
    {
        switch (pd_state_)
        {
        case PDState::LOADED:
            RCLCPP_INFO(this->get_logger(), "[PD] LOADED");
            if (!raise_sent_) {
                call_lift_service(LIFTSTATE::RAISE);
                raise_sent_ = true;
            }
            if (lift_state_ == LIFTSTATE::RAISE) {
                pd_state_ = PDState::RAISE;
                raise_sent_ = false;
            }
            break;

        case PDState::RAISE:
            RCLCPP_INFO(this->get_logger(), "[PD] RAISE");
            if (!pallet_aligned_) {
                call_align_trailer_service();
                pallet_aligned_ = true;
            }
            if (trailer_aligned()) {
                pd_state_ = PDState::ALIGN;
                pallet_aligned_ = false;
            }
            break;

        case PDState::ALIGN:
            RCLCPP_INFO(this->get_logger(), "[PD] ALIGN");
            if (!approach_sent_) {
                call_approach_service(trailer_distance_);
                approach_sent_ = true;
            }
            if (approach_reached()) {
                pd_state_ = PDState::APPROACH;
                approach_sent_ = false;
            }
            break;

        case PDState::APPROACH:
            RCLCPP_INFO(this->get_logger(), "[PD] APPROACH");
            if (!unload_sent_) {
                call_lift_service(LIFTSTATE::UNLOAD);  // or a specific delivery service
                unload_sent_ = true;
            }
            if (lift_state_ == LIFTSTATE::UNLOAD) {
                pd_state_ = PDState::UNLOAD;
                unload_sent_ = false;
            }
            break;

        case PDState::UNLOAD:
            RCLCPP_INFO(this->get_logger(), "[PD] UNLOAD");

            if (!backup_sent_){
                loaded_lift_ = false;
                call_approach_service(-trailer_distance_);
                backup_sent_ = true;
            }
            if (approach_reached()){
                pd_state_ = PDState::BACKUP;
                backup_sent_ = false;
            }
            break;

        case PDState::BACKUP:
            RCLCPP_INFO(this->get_logger(), "[PD] BACKUP");
            if (backup_completed()) {
                pd_state_ = PDState::IDLE;
            }
            break;

        case PDState::IDLE:
            RCLCPP_INFO(this->get_logger(), "[PD] IDLE → return to pallet search");
            state_ = State::PALLET_SEARCH;
            break;
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}
