#pragma once

#include "Camera.hpp"
#include "Cluster.hpp"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/projection_matrix.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <landmarks_msg/Landmarks_Msg.h>




#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>
#include <nav_msgs/Odometry.h>


struct ColorThresholder {

    std::string color;
    int index;
    std::vector<double> min_threshold;
    std::vector<double> max_threshold;

    image_transport::Publisher mask_pub;

    /**
     * @brief Thresholds the image with the given values and returns the size of the contour found
     * 
     * @param rgb 
     * @return std::shared_ptr<cv::Rect2d> The rect over the pole with largest area
     */
    std::shared_ptr<cv::Rect2d> thresholdImage(cv::Mat& rgb, int u, int v);



};


class ColourDifferentiator {

    public:
        ColourDifferentiator(ros::NodeHandle& nh_);

        /**
         * @brief Get the Best Color object
         * 
         * @param image_point (u, v) coordinates
         * @param rgb_image the image
         * @return std::string colour as a string
         */
        std::string getBestColor(cv::Point2d& image_point, cv::Mat& rgb_image);


    private:

        //params
        int pixel_crop;

        ros::NodeHandle nh;
        image_transport::ImageTransport it;



        std::vector<ColorThresholder> thresholders;
        std::vector<std::string> colours = { "red", "light_blue", "yellow",
            "green", "purple", "orange", "dark_blue", "black"};

};




template<typename PCLPoint>
void publish_pcl_cloud(typename pcl::PointCloud<PCLPoint>::Ptr pcl_pc, const ros::Publisher& pub, const std::string& frame_id) {
    sensor_msgs::PointCloud2 ros_pc;
    pcl::toROSMsg(*pcl_pc, ros_pc);
    ros_pc.header.frame_id = frame_id;
    pub.publish(ros_pc);
}

template<typename PCLPoint>
typename pcl::PointCloud<PCLPoint>::Ptr downsample_pc(const typename pcl::PointCloud<PCLPoint>::Ptr& cloud,
                                                                    const float leaf_size) {
    // Create the filter
    pcl::VoxelGrid<PCLPoint> voxel_grid_filter;
    voxel_grid_filter.setInputCloud(cloud);
    voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    //voxel_grid_filter.setMinimumPointsNumberPerVoxel(5);

    // Perform the filter
    typename pcl::PointCloud<PCLPoint>::Ptr filtered_cloud = typename pcl::PointCloud<PCLPoint>::Ptr(new typename pcl::PointCloud<PCLPoint>());
    voxel_grid_filter.filter(*filtered_cloud);

    return filtered_cloud;
}

class LandmarkPerception {

    typedef const sensor_msgs::ImageConstPtr ImageConst;
    typedef const sensor_msgs::PointCloud2ConstPtr PointCloud2Ptr;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;


    public:
        LandmarkPerception(ros::NodeHandle& nh_);
        void image_callback(ImageConst& raw_image, PointCloud2Ptr& point_cloud);


    private:
        std::vector<Cluster> get_clusters(bool publish_on, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_xyz_pc);
        pcl::PointXYZ calculate_centroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr estimateCurvature(const Cluster& cluster);

        std::string extract_colour(cv::Point2d& image_point, cv::Mat& rgb_image);

         // # Purple: 0, Orange: 1, Yellow: 2, Blue: 3,
        // # Green: 4, Red: 5, Black: 6, Turquoise: 7
        int colour_to_id(const std::string& color);



        ros::NodeHandle nh;
        message_filters::Subscriber<sensor_msgs::Image> raw_img_synch;
        // message_filters::Subscriber<sensor_msgs::Image> depth_img_synch;
        message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_synch;
        message_filters::Synchronizer<MySyncPolicy> sync;

        CameraPtr camera_ptr;

        //ros publishers
        ros::Publisher filter_point_cloud;
        ros::Publisher marker_pub;
        ros::Publisher landmark_pub;

        image_transport::ImageTransport it;
        image_transport::Publisher image_pub;


        ColourDifferentiator color_analysis;

        //params
        float downsample_leaf_size;
        int min_cluster_size;
        int max_cluster_size;
        double cluster_tolerance_factor;
        double cluster_min_z_threshold;
        double z_min;
        double z_max;
        double plane_segmentation_distance_threshold;
        double max_cluster_area;
        double std_dev_landmark_x;
        double std_dev_landmark_y;

};

// struct Mask {

//     cv::Point2i centroid;
//     double area;
//     std::string color;

// };
