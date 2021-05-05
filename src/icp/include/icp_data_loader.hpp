#ifndef ASS2_ICP_DATA_LOADER_HPP
#define ASS2_ICP_DATA_LOADER_HPP


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <tf2_ros/transform_listener.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

#include <memory>
#include <Eigen/Core>



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

//courtesy of Billiam Talbot
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

template<typename PCLPoint>
void publish_pcl_cloud(typename pcl::PointCloud<PCLPoint>::Ptr pcl_pc, const ros::Publisher& pub, const std::string& frame_id) {
    sensor_msgs::PointCloud2 ros_pc;
    pcl::toROSMsg(*pcl_pc, ros_pc);
    ros_pc.header.frame_id = frame_id;
    pub.publish(ros_pc);
}


template<typename PCLPoint>
typename pcl::PointCloud<PCLPoint>::Ptr filter_cloud_z(const typename pcl::PointCloud<PCLPoint>::Ptr& cloud,
                                                   double min_z, double max_z) {
    // Create the filter
    typename pcl::PassThrough<PCLPoint> pass;
    pass.setInputCloud(cloud);
    typename pcl::PointCloud<PCLPoint>::Ptr filtered_cloud = typename pcl::PointCloud<PCLPoint>::Ptr(new typename pcl::PointCloud<PCLPoint>());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min_z, max_z);
    pass.setFilterLimitsNegative (false);
    pass.filter(*filtered_cloud);
    return filtered_cloud;
}


// template<typename PCLPoint>
// typename pcl::PointCloud<PCLPoint>::Ptr rotate_cloud(const typename pcl::PointCloud<PCLPoint>::Ptr& cloud,
//                                                    const std::string& from_frame, const stf::string& to_frame) {
   
//     typename pcl::PointCloud<PCLPoint>::Ptr filtered_cloud = typename pcl::PointCloud<PCLPoint>::Ptr(new typename pcl::PointCloud<PCLPoint>());
//     pass.setFilterFieldName ("z");
//     pass.setFilterLimits (min_z, max_z);
//     pass.setFilterLimitsNegative (false);
//     pass.filter(*filtered_cloud);
//     return filtered_cloud;
// }

struct IcpData {
    PointCloudPtr point_cloud;
    tf2_msgs::TFMessage tf_static_messages;
    tf2_msgs::TFMessage tf_messages;

};

class IcpDataLoader {

    public:
        IcpDataLoader(std::string&& bag_name);
        std::shared_ptr<IcpData>& load_data();






    private:
        rosbag::Bag bag;
        std::shared_ptr<IcpData> data;
};


#endif
