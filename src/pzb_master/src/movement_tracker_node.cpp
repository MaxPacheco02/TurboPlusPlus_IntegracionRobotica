#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <iomanip>
#include <queue>
#include <iostream>
#include <vector>
#include <optional>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.hpp"
#include "std_srvs/srv/empty.hpp"

#include "pzb_msgs/msg/trailer_bearing.hpp"
#include "pzb_msgs/srv/movement_request_service.hpp"
#include "pzb_msgs/srv/trailer_align.hpp"
#include "pzb_msgs/srv/approach.hpp"

using namespace std::chrono_literals;

class MovementTrackerNode : public rclcpp::Node
{
public:
    MovementTrackerNode() : Node("movement_tracker_node")
    {
        using namespace std::placeholders;

        trailer_align_client_ = this->create_client<pzb_msgs::srv::TrailerAlign>("trailer_align_srv");
        pallet_align_client_ = this->create_client<std_srvs::srv::Empty>("pallet_align_srv");
        approach_client_ = this->create_client<pzb_msgs::srv::Approach>("approach_srv");

        movement_service_ = this->create_service<pzb_msgs::srv::MovementRequestService>(
            "movement_service", std::bind(
                                    &MovementTrackerNode::movement_srv_handle, this, _1, _2));

        rst_ref_id_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset_ref_id_service", std::bind(
                                        &MovementTrackerNode::reset_id_srv_handle, this, _1, _2));

        ref_id_pub_ = this->create_publisher<std_msgs::msg::Int64>("/movement_ref_id", 10);
        nav_ref_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry &msg)
            {
                pose_ = msg.pose.pose;
            });

        pid_e_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/pid_abs_err", 10,
            [this](const std_msgs::msg::Float64 &msg)
            {
                pid_err_ = msg.data;
            });

        vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist &msg)
            {
                vel_ = std::sqrt(
                    std::pow(msg.linear.x, 2) +
                    std::pow(msg.angular.z, 2));
            });

        update_timer_ = this->create_wall_timer(100ms, std::bind(&MovementTrackerNode::update, this));

        ref_id_msg.data = 0;

        // Add 10 seconds to latest update to avoid false positives in detection condition
        last_movement_update_ = this->get_clock()->now() - rclcpp::Duration(0, 10000 * 1e6);
    }

private:
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr ref_id_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_ref_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pid_e_sub_;

    rclcpp::Client<pzb_msgs::srv::TrailerAlign>::SharedPtr trailer_align_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr pallet_align_client_;
    rclcpp::Client<pzb_msgs::srv::Approach>::SharedPtr approach_client_;

    rclcpp::Service<pzb_msgs::srv::MovementRequestService>::SharedPtr movement_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr rst_ref_id_service_;

    rclcpp::TimerBase::SharedPtr update_timer_;

    rclcpp::Time last_movement_update_;

    std_msgs::msg::Int64 ref_id_msg;

    geometry_msgs::msg::Pose pose_, goal_;

    bool new_mov_ref{false};
    double pid_err_{0.0};
    double vel_{0};

    // Trailer aligning service call. Assumes the trailer is being detected.
    void movement_srv_handle(const std::shared_ptr<pzb_msgs::srv::MovementRequestService::Request> request,
                             std::shared_ptr<pzb_msgs::srv::MovementRequestService::Response> response)
    {
        last_movement_update_ = this->get_clock()->now();

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "MOVEMENT CLI: %s %d", request->type.c_str(), request->is_alignment_to_trailer);
        if (new_mov_ref || vel_ > 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Robot in movement");
            return;
        }

        if (request->type == "align")
        {
            if (request->is_alignment_to_trailer)
                call_trailer_cli(request->trailer_choice);
            else
                call_pallet_cli();
        }
        else
        {
            if (request->type == "nav")
            {
                geometry_msgs::msg::PoseStamped ref;
                ref.pose = request->ref_pose.pose;
                ref.header.frame_id = "odom";
                ref.header.stamp = this->get_clock()->now();

                nav_ref_pub_->publish(ref);
            }
            else // request->type == "approach"
                call_approach_cli(request->approach_distance);
        }

        new_mov_ref = true;
    }

    // Pallet aligning service call.
    void reset_id_srv_handle(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                             std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "RESET ID CLI");

        new_mov_ref = false;
        ref_id_msg.data = 0;
    }

    void call_trailer_cli(std::string trailer_choice)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "TRAILER CLI");

        auto request = std::make_shared<pzb_msgs::srv::TrailerAlign::Request>();
        request->trailer_type = trailer_choice;

        while (!trailer_align_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "trailer_align_client_ -> Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "trailer_align_client_ -> service not available, waiting again...");
        }

        auto result = trailer_align_client_->async_send_request(request);
        return;
    }

    void call_pallet_cli()
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "PALLET CLI");

        auto request = std::make_shared<std_srvs::srv::Empty::Request>();

        while (!pallet_align_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "pallet_align_client_ -> Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pallet_align_client_ -> service not available, waiting again...");
        }

        auto result = pallet_align_client_->async_send_request(request);
        return;
    }

    void call_approach_cli(double d)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "APPROACH CLI");

        auto request = std::make_shared<pzb_msgs::srv::Approach::Request>();
        request->distance = d;

        while (!approach_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "approach_client_ -> Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "approach_client_ -> service not available, waiting again...");
        }

        auto result = approach_client_->async_send_request(request);
        return;
    }

    void update()
    {
        // If a new movement was required, and the pzb is [almost] there, and it has already stopped.
        if (new_mov_ref && pid_err_ < 0.1 && vel_ == 0 && 
            ((this->get_clock()->now() - last_movement_update_) > rclcpp::Duration(0, 500 * 1e6))
        )
        {
            ref_id_msg.data++;
            new_mov_ref = false;
        }
        ref_id_pub_->publish(ref_id_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementTrackerNode>());
    rclcpp::shutdown();
    return 0;
}