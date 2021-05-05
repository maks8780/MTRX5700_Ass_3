#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>

class LaserScanReceiver
{

private:
     ros::NodeHandle n;
     std::string base_tf;
     std::string laser_topic;
     laser_geometry::LaserProjection projector;
     tf::TransformListener listener;
     message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
     tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;

     ros::Publisher pcl_pub;

public:
     sensor_msgs::PointCloud2 cloud_out;
     sensor_msgs::PointCloud2 cloud_in;
     pcl::PCLPointCloud2 pcl_in;
     pcl::PCLPointCloud2 pcl_out;
     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

     pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
     pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud;

     int x = 0;

     LaserScanReceiver(ros::NodeHandle node, std::string tf, std::string topic) : n(node),
                                                                                  base_tf(tf),
                                                                                  laser_topic(topic),
                                                                                  laser_sub(n, topic, 10),
                                                                                  laser_notifier(laser_sub, listener, tf, 10)

     {

          laser_notifier.registerCallback(&LaserScanReceiver::scanCallback, this);
          laser_notifier.setTolerance(ros::Duration(0.2));

          pcl_pub = node.advertise<sensor_msgs::PointCloud2>("pcl_in", 1);

          // point cloud pointers declared here
          current_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
          previous_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();


     }

     void scanCallback(const sensor_msgs::LaserScan scan_in)
     {


          // count for x, only performs ICP after call back has been called at
          // leasttwice
          x = x + 1;

          // transform scan to point cloud ------------------------s
          try
          {
               projector.transformLaserScanToPointCloud(
                   base_tf.c_str(), scan_in, cloud_in, listener);
          }
          catch (tf::TransformException &e)
          {
               ROS_ERROR("%s", e.what());
               return;
          }

          // ----------------------------------------------------

          ROS_INFO("publishing new point cloud");
          pcl_pub.publish(cloud_in);

          // converts sensor_msgs::point cloud
          pcl_conversions::toPCL(cloud_in, pcl_in);
          pcl::fromPCLPointCloud2(pcl_in, *current_cloud);

          if (x > 2)
          {
               // ICP ---------------------------------------------------------------------------
               ROS_INFO("PERFORMING ICP");

               // define input and output point clouds
               icp.setInputSource(current_cloud);

               icp.setInputTarget(previous_cloud);

               // Find iterative closest point
               pcl::PointCloud<pcl::PointXYZ> Final;
               icp.align(Final);
               ROS_INFO("HERE3");

               std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

               std::cout << icp.getFinalTransformation() << std::endl;

               // end ----------------------------------------------------------------------------
          }

          *previous_cloud = *current_cloud;
     }
};

int main(int argc, char **argv)
{

     ros::init(argc, argv, "laser_2_pointcloud");
     ros::NodeHandle node;

     std::string base_tf;
     std::string laser_topic;

     node.param("/laser_to_pointcloud/base_tf", base_tf, std::string("/odom"));
     node.param("/laser_to_pointcloud/laser_topic", laser_topic, std::string("/scan"));

     LaserScanReceiver laser(node, base_tf, laser_topic);

     ros::Rate loop_rate(10);
     while (node.ok())
     {
          ros::spinOnce();
          loop_rate.sleep();
     }
}