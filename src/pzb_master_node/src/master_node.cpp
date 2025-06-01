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

using namespace std::chrono_literals;

class MasterNode : public rclcpp::Node
{
public:
    MasterNode() : Node("state_machine"){
        using namespace std::placeholders;

        lift_sub_ = this->create_subscription<int>("/lift_state",10,
            [this](const int &msg){
                if(msg == 0){
                    loaded_lift = false;
                }else{
                    loaded_lift = true;
                }
            });

        pallet_sub_ = this->crceate_subscription<geometry_msgs::msg::PoseStamped>(
            "/pzb/pallet_pos",10,
            [this](const geometry_msgs::msg::PoseStamped &msg)
            {
                last_pallet_update_ = this->get_clock()->now();
                pallet_detected = true;
            });
        
        // trailer_sub_ = this->create_subscription<>();
        
            
        update_timer_ = this->create_wall_timer(10ms, std::bind(&MasterNode::state_switch, this));
        
        state = "feature_search";
        loaded_lift = false;
        pallet_detected = false;
        trailer_detected = false;

        last_pallet_update_ = this->get_clock().now();
    }

private:
    
    rclcpp::Subscription<int>::SharedPtr lift_sub_;
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
        
        // switch(state){
        //     case "feature search":
        //         if(pallet_detected && !loaded_lift){
        //             pallet_collection();
        //             state = "pallet_collection";
        //         }else{
        //             feature_search();
        //         }
        //     case "pallet_collection":
        //         if(loaded_lift){
        //             featur_search();
        //         }else{
        //             pallet_collection();
        //         }
        //     case "pallet_delivery":
        // }

        switch(loaded_lift){
            case false:
                if(pallet_detected){
                    pallet_collection();
                }else{
                    feature_search();
                }
                return;
            case true:
                if(trailer_detected){
                    pallet_delivery();
                }else{
                    frature_search();
                }
                return;
            default: 
                pallet_detected = false;
                trailer_detected = false;
                feature_search();
                return;
        }
    }

    void feature_search(){
        if (this->get_clock()->now() - last_pallet_update_ > rclcpp::Duration(0, 100 * 1e6))
            return;
    }

    void pallet_collection(){
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
