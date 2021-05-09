#include "Camera.hpp"
#include "CameraParams.hpp"


Camera::Camera(const CameraParamsPtr& camera_params_)
    :   camera_params(camera_params_) {}

 cv::Point3d Camera::projectPoints(const cv::Point2i& image_points, cv::Mat& depth_mat) {
    const float u = image_points.x;
    const float v = image_points.y;
    const float z = depth_mat.at<float>(u, v);

    const float x = (u- camera_params->intrinsics_.cu) * z * 1.0/camera_params->intrinsics_.fx;
    const float y = (v-camera_params->intrinsics_.cv) * z * 1.0/camera_params->intrinsics_.fy;

    cv::Point3d point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
}

cv::Point2d Camera::unprojectPoints(const cv::Point3d& world_points) {
    const float x = world_points.x;
    const float y = world_points.y;
    const float z = world_points.z;

    float u = (x * camera_params->intrinsics_.fx)/z + camera_params->intrinsics_.cu;
    float v = (y * camera_params->intrinsics_.fy)/z + camera_params->intrinsics_.cv;

    cv::Point2d point;
    point.x = u;
    point.y = v;
    return point;

}