#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

// ***************UNUSED AS OF NOW**********************

class CAM {
public:
    cv::Mat CamMat;
    cv::Mat CamMatN;
    cv::Mat CamTra;
    cv::Mat CamRot;
    cv::Mat CamDis;
    cv::Rect roi;

    cv::Mat CamRotW;
    cv::Mat CamTraW;

    std::string worldString;

    explicit CAM(const std::string& world) : worldString(world) {
        cameraCalibrate();
    }

    void cameraCalibrate() {
        cv::Size chessSize(5, 5);
        // float chessSq = 31.0f;
        cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 50, 0.001);

        std::vector<cv::Point3f> objp;
        for (int i = 0; i < chessSize.height; ++i)
            for (int j = 0; j < chessSize.width; ++j)
                objp.emplace_back(j, i, 0);

        for (int i = chessSize.width - 1; i < objp.size(); i += chessSize.width)
            std::reverse(objp.begin() + (i - (chessSize.width - 1)), objp.begin() + (i + 1));

        std::vector<std::vector<cv::Point3f>> objpoints;
        std::vector<std::vector<cv::Point2f>> imgpoints;

        std::string share_dir = ament_index_cpp::get_package_share_directory("aru_detection");
        std::string chessboard_path = share_dir + "/resources";

        std::vector<std::string> images;
        for (const auto& entry : std::filesystem::directory_iterator(chessboard_path)) {
            if (entry.path().extension() == ".png") {
                images.push_back(entry.path().string());
            }
        }

        int h = 0, w = 0, count = 0, worldIndex = 0;
        cv::Mat gray;

        for (const auto& fname : images) {
            cv::Mat img = cv::imread(fname);
            if (img.empty()) continue;

            h = img.rows;
            w = img.cols;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, chessSize, corners);
            if (found) {
                objpoints.push_back(objp);
                cv::cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1,-1), criteria);
                imgpoints.push_back(corners);

                if (fname.find(worldString) != std::string::npos)
                    worldIndex = count;

                count++;
                
                // cv::drawChessboardCorners(img, chessSize, corners, found);
                // cv::putText(img, fname.substr(fname.length() - 12), cv::Point(50, 50),
                //             cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 2);
                // cv::imshow("Imagen", img);
                // cv::waitKey(0);
            }
        }

        std::vector<cv::Mat> rvecs, tvecs;
        cv::Mat mtx, dist;
        cv::calibrateCamera(objpoints, imgpoints, gray.size(), mtx, dist, rvecs, tvecs);
        CamMat = mtx;
        CamDis = dist;
        CamMatN = cv::getOptimalNewCameraMatrix(mtx, dist, cv::Size(w, h), 1, cv::Size(w, h), &roi);

        CamRotW = rvecs[worldIndex];
        CamTraW = tvecs[worldIndex];
        std::cout << "CamMatN:\n" << CamMatN << std::endl;
        std::cout << "CamDis:\n" << CamDis << std::endl;
    }
};