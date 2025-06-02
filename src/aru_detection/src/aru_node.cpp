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
    size_t max_quat_history_ = 10; 

    cv::Mat CamMat;
    cv::Mat CamDis;

    cv::Rect roi;
    cv::Mat undistFrame;
    cv::Mat cropped;

    pzb_msgs::msg::ArucoMarker marker;

    bool nearest_pallet; 

    // MAIN LOOP
    void show(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat compressed_mat(msg->data); 
        cv::Mat frame = cv::imdecode(compressed_mat, cv::IMREAD_COLOR);

        // RECORTAR LA IMAGEN
        cv::undistort(frame, undistFrame, CamMat, CamDis);
        cropped = frame(roi);

        std::vector<std::tuple<cv::Vec3d, cv::Vec3d, int>> detections;
        std::tie(frame,detections) = aru_.pose_estimation(cropped, CamMat, CamDis);

        for (int i = 0; i < detections.size(); i++) {
            auto &[rvec, tvec, id] = detections[i];
            std::cout<< "tvec : " << tvec <<std::endl;
            std::cout<< "id : " << id <<std::endl;
            if (id != -1) {
                cv::Rodrigues(rvec, aruORI_);
                cv::cv2eigen(aruORI_, aruORI_eigen);
                aruPOS_ = Eigen::Vector3f(tvec[0], tvec[1], tvec[2]);
                
                if(id != 0){ // FOR LOCALIZATION
                    pubPosArray(id);
                    pubPosPallet();
                }else{ //FOR PALLETS
                    pubPosPallet();
                } 
            }       
            else {
                aruId_.clear();
            }
        }
        nearest_pallet = true;

        pos_pub_->publish(aruList_msg_);
        aruList_msg_.markers.erase(aruList_msg_.markers.begin() + 1, aruList_msg_.markers.end());
        cv::imshow("Imagen", frame);
        cv::waitKey(1);
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

        Eigen::Quaternionf q(aruORI_eigen);

        // TO SMOOTH OUT ROTATIONS
        // -------------------------------------------------------------------------------
        auto& q_deque = quat_history_[id];
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
        // -------------------------------------------------------------------------------

        marker.pose.orientation.x = q_avg.x() * scale_;
        marker.pose.orientation.y = q_avg.y() * scale_;
        marker.pose.orientation.z = q_avg.z() * scale_;
        marker.pose.orientation.w = q_avg.w() * scale_;

        aruList_msg_.markers.push_back(marker);
    }

    // PUBLISHER PARA ARUCO DE PALLET
    void pubPosPallet(){
        auto now = this->get_clock()->now();
        std::cout<<"in"<<std::endl;
        pose_stamped_msg_.header.stamp = now;
        pose_stamped_msg_.pose.position.x = aruPOS_[0] * scale_;
        pose_stamped_msg_.pose.position.y = aruPOS_[1] * scale_;
        pose_stamped_msg_.pose.position.z = aruPOS_[2] * scale_;
        Eigen::Quaternionf q(aruORI_eigen);
        pose_stamped_msg_.pose.orientation.x = q.x();
        pose_stamped_msg_.pose.orientation.y = q.y();
        pose_stamped_msg_.pose.orientation.z = q.z();
        pose_stamped_msg_.pose.orientation.w = q.w();
        pallet_pub_->publish(pose_stamped_msg_);
    }

    // SUBJECT TO CHANGE, NO REASON LEFT NOT TO PUBLISH AS STATIC TF IN URDF
    // ----------------------------------------------------------------------------
    void calCamSys() {
        quat.setRPY(1.5708f, -M_PI, -M_PI - 1.5708f);
        // 0.0872665+
        poseW_.id = -2;
        poseW_.pose.position.x = 0.108;
        poseW_.pose.position.y = 0.0;
        poseW_.pose.position.z = 0.044;
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



// // count: 230
// CamMat = (cv::Mat_<double>(3,3) << 640.19980366,0.0,356.53969343,
//                                     0.0,631.41159734,241.09692846,
//                                     0.0,0.0,1.0);

// CamDis = (cv::Mat_<double>(1,5) << 0.10816224,0.02314439,0.0348719,0.01768096,-0.37449394);
// roi = cv::Rect(18, 13, 614, 338);


// // ONLY RIGHT SIDE FUCKED UP
// CamMat = (cv::Mat_<double>(3,3) << 531.16390426,0.0,291.20749879,
//                                             0.0,532.68741633,214.26092426,
//                                             0.0,0.0,1.0);

// CamDis = (cv::Mat_<double>(1,5) << 0.0106159534, 0.0619105354, 0.00180013372, -0.00799441912,-3.90060999);
// roi = cv::Rect(33, 56, 520, 292);


// // PARAMETROS DE CAMARA DE MI COMPU 
// CamMat = (cv::Mat_<double>(3,3) << 641.32642765,0.0,299.43970036,
//                                            0.0,639.79591303,342.58997503,
//                                            0.0,0.0,1.0);

// CamDis = (cv::Mat_<double>(1,5) << 0.08207287,-0.73105371,0.036685,-0.00448076,1.29510307);


// ALL 100000000000 FOTOS
// CamMat = (cv::Mat_<double>(3,3) << 469.87172695,0.0,314.5588278,
//                                            0.0,597.02651066,160.386131,
//                                            0.0,0.0,1.0);

// CamDis = (cv::Mat_<double>(1,5) << 0.05792354,1.47119549,-0.02989828,-0.008226,-6.79787084);