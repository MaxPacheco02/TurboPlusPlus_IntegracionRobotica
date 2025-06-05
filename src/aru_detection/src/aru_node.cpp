#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <cmath>
#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include <tf2/utils.h>

#include "sensor_msgs/msg/image.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/core/eigen.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pzb_msgs/msg/aruco_marker.hpp"
#include "pzb_msgs/msg/aruco_list.hpp"

#include "sensor_msgs/msg/compressed_image.hpp"

#include "aru_detection/POS_ARU.hpp"  
#include "aru_detection/POS_IM.hpp"  

class ObjPos : public rclcpp::Node {
    
public:
    ObjPos() : Node("ObjPos"), aru_() {
        RCLCPP_INFO(this->get_logger(), "DETECCION DE CAJAS");

        using namespace std::placeholders;

        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/signal_frame", 10,
        std::bind(&ObjPos::show, this, _1));

        pos_pub_ = this->create_publisher<pzb_msgs::msg::ArucoList>("/pzb/aru_sys", 10);
        aruList_msg_.markers.reserve(20);
        
        pallet_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pzb/pallet_pos", 10);

        aru_.detectorSetup();

        wayPointD_ = 100.0;
        scale_ = 0.001f;
        
        // PARAMETROS DE LA CAMARA DEL PZB
        CamMat = (cv::Mat_<double>(3,3) << 640.19980366,0.0,356.53969343,
                                           0.0,631.41159734,241.09692846,
                                           0.0,0.0,1.0);

        CamDis = (cv::Mat_<double>(1,5) << 0.10816224,0.02314439,0.0348719,0.01768096,-0.37449394);

        roi = cv::Rect(18, 13, 614, 338);

        aruList_msg_.header.frame_id = "camera";
        pose_stamped_msg_.header.frame_id = "camera";

        nearest_pallet = true;
        
        calCamSys();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_; 
    rclcpp::Publisher<pzb_msgs::msg::ArucoList>::SharedPtr pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pallet_pub_;
    pzb_msgs::msg::ArucoList aruList_msg_;
    pzb_msgs::msg::ArucoMarker poseW_;

    geometry_msgs::msg::PoseStamped pose_stamped_msg_;

    ARU aru_;

    tf2::Quaternion quat;

    cv::Mat aruORI_;
    Eigen::Matrix3f aruORI_eigen;
    Eigen::Vector3f aruPOS_;
    float wayPointD_;
    float scale_;

    std::unordered_map<int, POINTS> aruId_;

    std::unordered_map<int, std::deque<Eigen::Quaternionf>> quat_history_;
    size_t max_quat_history_{10}; 
    const float max_jump_rad{1.0f}; 

    cv::Mat CamMat, CamDis;

    cv::Rect roi;
    cv::Mat undistFrame, cropped;

    pzb_msgs::msg::ArucoMarker marker;

    bool nearest_pallet; 

    // MAIN LOOP
    void show(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat compressed_mat(msg->data); 
        cv::Mat frame = cv::imdecode(compressed_mat, cv::IMREAD_COLOR);

        // cv::VideoCapture cap(0);

        // while (rclcpp::ok()) {
        //     cv::Mat frame;
        //     if (!cap.read(frame)) continue;

        // RECORTAR LA IMAGEN
        cv::undistort(frame, undistFrame, CamMat, CamDis);
        cropped = frame(roi);

        std::vector<std::tuple<cv::Vec3d, cv::Vec3d, int>> detections;
        std::tie(frame,detections) = aru_.pose_estimation(cropped, CamMat, CamDis);

        for (int i = 0; i < (int)detections.size(); i++) {
            auto &[rvec, tvec, id] = detections[i];
            std::cout<< "tvec : " << tvec <<std::endl;
            if (id != -1) {
                cv::Rodrigues(rvec, aruORI_);
                cv::cv2eigen(aruORI_, aruORI_eigen);
                aruPOS_ = Eigen::Vector3f(tvec[0], tvec[1], tvec[2]);
                
                if(id != 0){ // FOR LOCALIZATION
                    pubPosArray(id);
                }else if(nearest_pallet == true){ //FOR PALLETS
                    pubPosPallet();
                    nearest_pallet = false;
                } 
            }       
            else {
                aruId_.clear();
            }
        }
        nearest_pallet = true;

        pos_pub_->publish(aruList_msg_);

        if (aruList_msg_.markers.size() > 1) {
            aruList_msg_.markers.erase(aruList_msg_.markers.begin() + 1, aruList_msg_.markers.end());
        }
        cv::imshow("Imagen", frame);
        cv::waitKey(1);
        // }
    }

    // PUBLISH ARUCOS FOR LOCALIZATION
    void pubPosArray(int id) {
        auto now = this->get_clock()->now();
        aruList_msg_.header.stamp = now;

        // pzb_msgs::msg::ArucoMarker marker;
        marker.id = id;

        if(aruId_.find(id) == aruId_.end()) {
            aruId_.emplace(id, POINTS());
        }

        POINTS& smoother = aruId_[id];

        marker.pose.position.x = smoother.ma_get(aruPOS_[0], 0) * scale_;
        marker.pose.position.y = smoother.ma_get(aruPOS_[1], 1) * scale_;
        marker.pose.position.z = smoother.ma_get(aruPOS_[2], 2) * scale_;

        // TO SMOOTH OUT ROTATIONS WITH SLERP (APPARENTLY)
        auto& q_deque = quat_history_[id];
        Eigen::Quaternionf q(aruORI_eigen);
        Eigen::Quaternionf q_avg = slerp_calc(q_deque, q);

        marker.pose.orientation.x = q_avg.x();
        marker.pose.orientation.y = q_avg.y();
        marker.pose.orientation.z = q_avg.z();
        marker.pose.orientation.w = q_avg.w();

        aruList_msg_.markers.push_back(marker);
    }

    // PUBLISHER PARA ARUCO DE PALLET
    void pubPosPallet(){

        if(aruId_.find(0) == aruId_.end()) {
            aruId_.emplace(0, POINTS());
        }
        POINTS& smoother = aruId_[0];

        auto& q_deque = quat_history_[0];
        Eigen::Quaternionf q(aruORI_eigen);
        Eigen::Quaternionf q_avg = slerp_calc(q_deque, q);

        pose_stamped_msg_.header.stamp = this->get_clock()->now();
        pose_stamped_msg_.pose.position.x = smoother.ma_get(aruPOS_[0],0) * scale_;
        pose_stamped_msg_.pose.position.y = smoother.ma_get(aruPOS_[1],1) * scale_;
        pose_stamped_msg_.pose.position.z = smoother.ma_get(aruPOS_[2],2) * scale_;
        pose_stamped_msg_.pose.orientation.x = q_avg.x();
        pose_stamped_msg_.pose.orientation.y = q_avg.y();
        pose_stamped_msg_.pose.orientation.z = q_avg.z();
        pose_stamped_msg_.pose.orientation.w = q_avg.w();
        pallet_pub_->publish(pose_stamped_msg_);
    }


    Eigen::Quaternionf slerp_calc(std::deque<Eigen::Quaternionf> &q_deque, const Eigen::Quaternionf& q){
        // Check if current value is too far from the previous
        if (!q_deque.empty()) {
            float dot_product = q_deque.back().dot(q);
            dot_product = std::clamp(dot_product, -1.0f, 1.0f);
            float angle_diff = 2.0f * std::acos(std::abs(dot_product));

            if (angle_diff > max_jump_rad) {
                q_deque.clear();  // RESET
            }
        }
        if (q_deque.size() >= max_quat_history_) {
            q_deque.pop_front();
        }
        q_deque.push_back(q);
        // SLERP average
        Eigen::Quaternionf q_avg = q_deque.front();
        for (size_t i = 1; i < q_deque.size(); ++i) {
            float t = 1.0f / static_cast<float>(i + 1);  
            Eigen::Quaternionf q_other = q_deque[i];

            if (q_avg.dot(q_other) < 0.0f) {
                q_other.coeffs() *= -1.0f;  
            }
            q_avg = q_avg.slerp(t, q_other);
        }
        q_avg.normalize();
        return q_avg;
    }


    // SUBJECT TO CHANGE, NO REASON LEFT NOT TO PUBLISH AS STATIC TF IN URDF
    // ----------------------------------------------------------------------------
    void calCamSys() {
        quat.setRPY(1.5708f, -M_PI, -M_PI - 1.5708f);
        poseW_.id = -2;
        poseW_.pose.position.x = 0.056;
        poseW_.pose.position.y = 0.0;
        poseW_.pose.position.z = 0.195;
        poseW_.pose.orientation.x = quat[3];
        poseW_.pose.orientation.y = quat[2];
        poseW_.pose.orientation.z = quat[1];
        poseW_.pose.orientation.w = quat[0];
        aruList_msg_.markers.push_back(poseW_);
    }
    // ----------------------------------------------------------------------------
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ObjPos>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 1;
}
