#include <deque>
#include <unordered_map>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"

class ARU {
public:
    ARU() {
        aru_ROT_ = cv::Mat::zeros(3, 1, CV_64F);
        aru_TRAN_ = cv::Mat::zeros(3, 1, CV_64F);
        markerLength_ = 101.0;

        objp_ = (cv::Mat_<float>(4,3) <<
            -markerLength_/2,  markerLength_/2, 0,
             markerLength_/2,  markerLength_/2, 0,
             markerLength_/2, -markerLength_/2, 0,
            -markerLength_/2, -markerLength_/2, 0);
    }

    void detectorSetup() {
        auto aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        cv::aruco::DetectorParameters parameters;
        detector_ = std::make_unique<cv::aruco::ArucoDetector>(aruco_dict, parameters);
    }

    std::tuple<cv::Mat, std::vector<std::tuple<cv::Vec3d, cv::Vec3d, int>>> pose_estimation(cv::Mat& frame,
                                                                       const cv::Mat& camMatrix,
                                                                       const cv::Mat& distCoeffs) {
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::tuple<cv::Vec3d, cv::Vec3d, int>> result;

        detector_->detectMarkers(gray, corners, ids);

        if (!corners.empty()) {
            for (int i = 0; i < ids.size(); i++) {
                int marker_id = ids[i];
                aruSeen_[marker_id].push_back(true);
                
                if (aruSeen_[marker_id].size() > 1 && std::all_of(aruSeen_[marker_id].begin(), aruSeen_[marker_id].end(), [](bool v){ return v; })) {
                    cv::Vec3d rvec, tvec;
                    cv::solvePnP(objp_, corners[i], camMatrix, distCoeffs, rvec, tvec);

                    cv::aruco::drawDetectedMarkers(frame, corners, ids);
                    cv::drawFrameAxes(frame, camMatrix, distCoeffs, rvec, tvec, 20.0);

                    aru_ROT_ = cv::Mat(rvec);
                    aru_TRAN_ = cv::Mat(tvec);

                    result.emplace_back(rvec, tvec, marker_id);
                }
            }
        } else {
            result.clear();
            result.emplace_back(cv::Vec3d(0,0,0), cv::Vec3d(0,0,0), -1);
            aruSeen_.clear();
        }

        return {frame, result};
    }

    cv::Mat aru_ROT_, aru_TRAN_;
    double markerLength_;
    cv::Mat objp_;

private:
    std::unique_ptr<cv::aruco::ArucoDetector> detector_;
    std::unordered_map<int, std::deque<bool>> aruSeen_;
};