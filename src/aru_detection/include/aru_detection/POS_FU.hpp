#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

// ***************UNUSED AS OF NOW**********************

class POS {
public:
    POS(const cv::Mat& CamRotW, const cv::Mat& CamTraW, const cv::Mat& CamMat) {
        cv::Rodrigues(CamRotW, R_);
        T_ = CamTraW;
        K_ = CamMat;

        cv::hconcat(R_, T_, H_temp_);
        H_ = cv::Mat::eye(4, 4, CV_64F);
        H_temp_.copyTo(H_(cv::Rect(0, 0, 4, 3)));

        cv::hconcat(R_.t(), -R_.t() * T_, H_inv_temp_);
        H_inv_ = cv::Mat::eye(4, 4, CV_64F);
        H_inv_temp_.copyTo(H_inv_(cv::Rect(0, 0, 4, 3)));

        Pw_ = (cv::Mat_<double>(4,1) << 0, 0, 0, 0);
        Rw_ = cv::Mat::eye(3, 3, CV_64F);

        Tg_ = T_.clone();
        Rg_ = R_.clone();

        Rc_ = cv::Mat::zeros(3, 3, CV_64F);
        Pc_ = cv::Mat::zeros(4, 1, CV_64F);
        Rg_inv_ = Rg_.inv();
    }

    std::pair<cv::Mat, cv::Mat> worldCal() {
        cv::Mat one = (cv::Mat_<double>(1, 1) << 1.0);
        cv::Mat Tg;
        cv::vconcat(Tg_, one, Tg);
        Rc_ = Rg_ * Rw_;
        Pc_ = Pw_ + Tg;
        return {Rg_, Pc_};
    }

    // cv::Mat objCal(const cv::Mat& PUV, double Zc) {
    //     return planeToWorld(R_, T_, K_, PUV, Zc);
    // }

    // std::vector<int> planCal(const cv::Mat& W) {
    //     return worldToPlane(W, H_, K_);
    // }

    // std::vector<int> worldToPlane(const cv::Mat& W, const cv::Mat& H, const cv::Mat& K) {
    //     cv::Mat M;
    //     cv::hconcat(K, cv::Mat::zeros(3,1,CV_64F), M);
    //     cv::Mat C = H * W;
    //     cv::Mat V = M * C;
    //     int u = static_cast<int>(std::round(V.at<double>(0) / V.at<double>(2)));
    //     int v = static_cast<int>(std::round(V.at<double>(1) / V.at<double>(2)));
    //     return {u, v};
    // }

    // cv::Mat planeToWorld(const cv::Mat& R, const cv::Mat& Tp, const cv::Mat& K, const cv::Mat& PUV, double Zc) {
    //     cv::Mat C_norm = K.inv() * PUV;
    //     cv::Mat C = Zc * C_norm;
    //     cv::vconcat(C, cv::Mat::ones(1, 1, CV_64F), C);
    //     cv::Mat R_Tp, H_inv;
    //     cv::hconcat(R.t(), -R.t()*Tp, R_Tp);
    //     H_inv = cv::Mat::eye(4, 4, CV_64F);
    //     R_Tp.copyTo(H_inv(cv::Rect(0, 0, 4, 3)));
    //     return H_inv * C;
    // }

    // cv::Mat aruOriToWorld(const cv::Mat& R_aru) {
    //     cv::Mat R_cv;
    //     cv::Rodrigues(R_aru, R_cv);
    //     return R_cv * Rc_.inv();
    // }

    // cv::Mat aruTranToWorld(const cv::Mat& T_aru) {
    //     cv::Mat T_aug;
    //     cv::vconcat(T_aru, cv::Mat::ones(1,1,CV_64F), T_aug);
    //     return H_inv_ * T_aug;
    // }

private:
    cv::Mat R_, T_, K_;
    cv::Mat H_, H_inv_, H_temp_, H_inv_temp_;
    cv::Mat Pw_, Pc_, Rc_, Tg_, Rg_, Rg_inv_;
    cv::Mat Rw_;
};