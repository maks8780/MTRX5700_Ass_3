// THis file receives and can be used to load data tf and point cloud
// data from the recorded ros bag files

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <vector>
#include "icp_data_loader.hpp"


IcpDataLoader::IcpDataLoader(std::string&& bag_name) {
    bag.open(bag_name, rosbag::bagmode::Read);
    ROS_INFO_STREAM("Loading data from bagfile " << bag_name);

}

std::shared_ptr<IcpData>& IcpDataLoader::load_data() {
    data = std::make_shared<IcpData>();

    std::vector<std::string> topics = {"/tf_static", "/tf", "/realsense/depth/points"};
    tf2_ros::TransformBroadcaster br;
    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m: view) {
        const std::string& msg_topic = m.getTopic();


        sensor_msgs::PointCloud2ConstPtr msg_ptr = m.instantiate<sensor_msgs::PointCloud2>();

        if (msg_ptr != nullptr && msg_topic == "/realsense/depth/points") {
            sensor_msgs::PointCloud2 msg = *msg_ptr;

            PointCloudPtr mesh_pcl_pc(new PointCloud());
            pcl::fromROSMsg(msg, *mesh_pcl_pc);
            data->point_cloud = mesh_pcl_pc;
        }

        tf2_msgs::TFMessageConstPtr tf_msg_ptr = m.instantiate<tf2_msgs::TFMessage>();
        if (tf_msg_ptr != nullptr && (msg_topic == "/tf" || msg_topic == "/tf_static")) {
            if (msg_topic == "/tf") {
                data->tf_messages = *tf_msg_ptr;
                for(auto& transforms : tf_msg_ptr->transforms) {
                    br.sendTransform(transforms);
                    ros::spinOnce();

                }
            }
            else if (msg_topic == "/tf_static") {
                data->tf_static_messages = *tf_msg_ptr;
                for(auto& transforms : tf_msg_ptr->transforms) {
                    static_broadcaster.sendTransform(transforms);
                    ros::spinOnce();

                }
            }
            
        }
        ros::spinOnce();

    }
    bag.close();
    return data;
}