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
    MasterNode() : Node("state_machine"){
        using namespace std::placeholders;

        lift_sub_ = this->create_subscription<pzb_msgs::msg::LiftState>("/lift_state",10,
            [this](const pzb_msgs::msg::LiftState &msg){

            });
         
        lift_client_ = this->create_client<pzb_msgs::srv::LiftService>("lift_service");

        pallet_sub_ = this->crceate_subscription<geometry_msgs::msg::PoseStamped>(
            "/pzb/pallet_pos",10,
            [this](const geometry_msgs::msg::PoseStamped &msg)
            {
                last_pallet_update_ = this->get_clock()->now();
                pallet_detected = true;
            });
            
        // trailer_sub_ = this->create_subscription<>();        
            
        update_timer_ = this->create_wall_timer(10ms, std::bind(&MasterNode::state_switch, this));
        
        state = "pallet_search";
        loaded_lift = false;
        pallet_detected = false;
        trailer_detected = false;

        last_pallet_update_ = this->get_clock().now();
    }

private:
    
    rclcpp::Subscription<pzb_msgs::msg::LiftState>::SharedPtr lift_sub_;
    rclcpp::Client<pzb_msgs::srv::LiftService>::SharedPtr lift_client_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_sub_;
    // rclcpp::Subscription<>::SharedPtr trailer_sub_;
    rclcpp::TimerBase::SharedPtr update_timer_;

    bool loaded_lift;
    bool pallet_detected;
    bool trailer_detected;

    std::string state;

    rclcpp::Time last_pallet_update_;

    // MAQUINA DE TRES ESTADOS
    void state_switch(){
        
        switch(state){
            case "pallet_search":
                if(pallet_detected && !loaded_lift){
                    pallet_collection();
                    state = "pallet_collection";
                }else{
                    pallet_search();
                }
            case "pallet_collection":
                if(loaded_lift){
                    trailer_search();
                }else{
                    pallet_collection();
                }
            case "trailer_search":
                if (trailer_detected && loaded_lift){
                    pallet_delivery();
                }else{
                    trailer_search();
                }
            case "pallet_delivery":
                if(!loaded_lift){
                    pallet_search();
                }else{
                    pallet_delivery();
                }   
            default:
                pallet_search();
                trailer_detected = false;
                pallet_detected = false;
                loaded_lift = false;
        }
    }

    void pallet_search(){
        return;
    }

    void pallet_collection(){
        return; 
    }

    void trailer_search(){
        return;
    }

    void pallet_delivery(){
        return;
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MasterNode>());
    rclcpp::shutdown();
    return 0;
}


// if (this->get_clock()->now() - last_pallet_update_ > rclcpp::Duration(0, 100 * 1e6))
//     return;