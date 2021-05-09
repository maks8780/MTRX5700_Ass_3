#pragma once


#include "CameraParams.hpp"
#include <Eigen/Core>

#include <opencv2/core.hpp>


class Camera {

    public:
        Camera(const CameraParamsPtr& camera_params_);

        /**
         * @brief Takes points on image place (u, v) and projects them 
         * using the depth map.
         * 
         * @param image_points cv::Point2d -> points u, v
         * @param depth_mat 
         * @return cv::Point3d (x,y,z) in 3D
         */
        cv::Point3d projectPoints(const cv::Point2i& image_points, cv::Mat& depth_mat);

        /**
         * @brief Takes point in 3d space and projects it into the camera frame
         * 
         * @param world_points x, y, z in robot/world coordinates
         * @return cv::Point2d (u, v) in the camera plane
         */
        cv::Point2d unprojectPoints(const cv::Point3d& world_points);


        inline const std::string getCameraLink() const {return camera_params->camera_link_; };



    private:
        CameraParamsPtr camera_params;
};

typedef std::shared_ptr<Camera> CameraPtr;