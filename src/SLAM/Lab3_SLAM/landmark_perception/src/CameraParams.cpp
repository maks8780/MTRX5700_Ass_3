#include "CameraParams.hpp"

#include <sensor_msgs/CameraInfo.h>


CameraParams::CameraParams(const sensor_msgs::CameraInfo& camera_info_)
    :   camera_model_(camera_info_.distortion_model),
        camera_link_(camera_info_.header.frame_id),
        image_size_(cv::Size(camera_info_.width, camera_info_.height)) {

    intrinsics_.fx = camera_info_.K[0];
    intrinsics_.fy = camera_info_.K[4];
    intrinsics_.cu = camera_info_.K[2];
    intrinsics_.cv = camera_info_.K[5];

}