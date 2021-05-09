#pragma once

#include <string>
#include <vector>
#include <memory>

#include <opencv2/core/core.hpp>
#include <sensor_msgs/CameraInfo.h>


struct Intrinsics {
    double fx, fy, cu, cv;
};


class CameraParams {

    public:
        CameraParams(const sensor_msgs::CameraInfo& camera_info_);

        //! Camera model: pinhole, etc
        std::string camera_model_;
        std::string camera_link_;

        //! fu, fv, cu, cv
        Intrinsics intrinsics_;
        //! OpenCV structures: needed to compute the undistortion map.
        //! 3x3 camera matrix K (last row is {0,0,1})
        cv::Mat K_;

        cv::Size image_size_;

        //! Distortion parameters
        std::vector<double> distortion_coeff_;
        cv::Mat distortion_coeff_mat_;

};

typedef std::shared_ptr<CameraParams> CameraParamsPtr;

