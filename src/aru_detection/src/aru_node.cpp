#include "rclcpp/rclcpp.hpp"
#include "aru_detection/POS_NOD.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ObjPos>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 1;
}