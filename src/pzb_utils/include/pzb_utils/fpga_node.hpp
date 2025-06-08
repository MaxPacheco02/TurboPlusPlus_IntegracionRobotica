// UBICACIÓN: ~/pzb_ws/src/pzb_utils/include/pzb_utils/fpga_node.hpp
// ARCHIVO OPCIONAL - Solo si quieres separar header de implementación

#ifndef PZB_UTILS_FPGA_NODE_HPP_
#define PZB_UTILS_FPGA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <pzb_msgs/msg/lift_state.hpp>
#include "pzb_msgs/srv/set_lift.hpp"
#include <serial/serial.h>
#include <thread>
#include <mutex>
#include <map>

class FPGANode : public rclcpp::Node
{
public:
    FPGANode();
    ~FPGANode();

private:
    void initializeMappings();
    void serialReadingLoop();
    void publishCurrentState();
    void setLiftCallback(const std::shared_ptr<pzb_msgs::srv::SetLift::Request> request,
                        std::shared_ptr<pzb_msgs::srv::SetLift::Response> response);
    std::string getStateName(int8_t state);
    
    // Miembros privados
    std::unique_ptr<serial::Serial> serial_port_;
    rclcpp::Publisher<pzb_msgs::msg::LiftState>::SharedPtr state_publisher_;
    rclcpp::Service<pzb_msgs::srv::SetLift>::SharedPtr set_lift_service_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    std::thread serial_reading_thread_;
    std::mutex state_mutex_;
    int8_t current_state_;
    
    // Mapeos
    std::map<char, int8_t> fpga_to_ros_state_;
    std::map<int8_t, char> ros_to_fpga_command_;
    std::map<int8_t, std::string> state_names_;
};

#endif  // PZB_UTILS_FPGA_NODE_HPP_