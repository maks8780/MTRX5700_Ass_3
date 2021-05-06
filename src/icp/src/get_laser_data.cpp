#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

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
     ros::Publisher pcl_pub_in;
     ros::Publisher icp_odom_pub;
     nav_msgs::Odometry last_odom;
     ros::Subscriber odom_sub;

public:
     sensor_msgs::PointCloud2 cloud_out;
     sensor_msgs::PointCloud2 cloud_in;
     pcl::PCLPointCloud2 pcl_in;
     pcl::PCLPointCloud2 pcl_out;
     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
     pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud;
     pcl::PointCloud<pcl::PointXYZ>::Ptr previous_cloud;
     Eigen::Matrix4f current_pose_hom;
     std::ofstream stamped_groundtruth;
     std::ofstream stamped_traj_estimate;
     bool is_first = true;

     LaserScanReceiver(ros::NodeHandle node, std::string tf, std::string topic) : n(node),
                                                                                  base_tf(tf),
                                                                                  laser_topic(topic),
                                                                                  laser_sub(n, topic, 100),
                                                                                  laser_notifier(laser_sub, listener, tf, 100)

     {

          laser_notifier.registerCallback(&LaserScanReceiver::scanCallback, this);
          laser_notifier.setTolerance(ros::Duration(0.2));

          // groundtruth odom subscriber declaration 
          odom_sub = node.subscribe<nav_msgs::Odometry>("/odom", 100, &LaserScanReceiver::odomCallback, this);

          // input pointcloud publisher
          pcl_pub_in = node.advertise<sensor_msgs::PointCloud2>("pcl_in", 1);

          // icp odom publisher to be plotted in real time
          icp_odom_pub = node.advertise<nav_msgs::Odometry>("icp_odom", 10);

          // pointcloud pointer declarations
          current_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
          previous_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
          nav_msgs::OdometryConstPtr odom_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");

          // setting initial homogenous position matrix to the initial
          // odom's position
          tf::Quaternion inital_quaternion;
          inital_quaternion.setX(odom_ptr->pose.pose.orientation.x);
          inital_quaternion.setY(odom_ptr->pose.pose.orientation.y);
          inital_quaternion.setZ(odom_ptr->pose.pose.orientation.z);
          inital_quaternion.setW(odom_ptr->pose.pose.orientation.w);

          // < quaternion -> rotation Matrix
          tf::Matrix3x3 initial_rot;
          initial_rot.setRotation(inital_quaternion);

          current_pose_hom = Eigen::Matrix4f(4, 4);
          current_pose_hom << initial_rot[0][0], initial_rot[0][1], initial_rot[0][2], odom_ptr->pose.pose.position.x,
              initial_rot[1][0], initial_rot[1][1], initial_rot[1][2], odom_ptr->pose.pose.position.y,
              initial_rot[2][0], initial_rot[2][1], initial_rot[2][2], odom_ptr->pose.pose.position.z,
              0, 0, 0, 1;

          stamped_traj_estimate.open("/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/results_folder/stamped_traj_estimate.txt");
          stamped_groundtruth.open("/home/mtrx5700/Desktop/MTRX5700/MTRX5700_Ass_3/results_folder/stamped_groundtruth.txt");
     }

     void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
     {
          last_odom = *odom_msg;
     }

     void scanCallback(const sensor_msgs::LaserScan &scan_in)
     {
          static tf2_ros::TransformBroadcaster tf_broadcaster;
          geometry_msgs::TransformStamped transform_stamped;

          // transform scan to point cloud ------------------------s

          projector.transformLaserScanToPointCloud(
              base_tf.c_str(), scan_in, cloud_in, listener);

          // ----------------------------------------------------

          // publishing pc converted from laser scan
          pcl_pub_in.publish(cloud_in);

          // converts sensor_msgs::pointCloud to pcl::PointXYZ
          // for icp processing
          pcl_conversions::toPCL(cloud_in, pcl_in);
          pcl::fromPCLPointCloud2(pcl_in, *current_cloud);

          // ICP ---------------------------------------------------------------------------
          ROS_INFO("PERFORMING ICP");
          if (is_first)
          {
               *previous_cloud = *current_cloud;
               is_first = false;
               return;
          }

          // define input and output point clouds
          icp.setInputSource(current_cloud);
          icp.setInputTarget(previous_cloud);

          // maximum correspondance distance of 0.1m
          icp.setMaxCorrespondenceDistance(0.4f);

          // Set the transformation epsilon
          icp.setTransformationEpsilon(1e-9);

          // icp.setMaximumIterations(500);
          // icp.setEuclideanFitnessEpsilon(1);
          // icp.setRANSACOutlierRejectionThreshold(1.5);

          // Find iterative closest point
          pcl::PointCloud<pcl::PointXYZ> Final;
          icp.align(Final);

          std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

          Eigen::Matrix4f transformation = icp.getFinalTransformation();

          std::cout << icp.getFinalTransformation() << std::endl;

          current_pose_hom = current_pose_hom * transformation;

          tf2::Matrix3x3 rotation_mat(current_pose_hom(0, 0), current_pose_hom(0, 1), current_pose_hom(0, 2), current_pose_hom(1, 0), current_pose_hom(1, 1), current_pose_hom(1, 2), current_pose_hom(2, 0), current_pose_hom(2, 1), current_pose_hom(2, 2));
          tf2::Vector3 translation_mat(current_pose_hom(0, 3), current_pose_hom(1, 3), current_pose_hom(2, 3));
          tf2::Transform trans_tf(rotation_mat, translation_mat);

          transform_stamped.header.stamp = scan_in.header.stamp;
          transform_stamped.header.frame_id = "/base_footprint";
          transform_stamped.child_frame_id = "transformed_frame";
          transform_stamped.transform = tf2::toMsg(trans_tf);
          tf_broadcaster.sendTransform(transform_stamped);

          nav_msgs::Odometry odom;
          odom.header.stamp = scan_in.header.stamp;
          odom.header.frame_id = "/base_footprint";
          odom.child_frame_id = "transformed_frame";

          odom.pose.pose.orientation.x = transform_stamped.transform.rotation.x;
          odom.pose.pose.orientation.y = transform_stamped.transform.rotation.y;
          odom.pose.pose.orientation.z = transform_stamped.transform.rotation.z;
          odom.pose.pose.orientation.w = transform_stamped.transform.rotation.w;

          odom.pose.pose.position.x = transform_stamped.transform.translation.x;
          odom.pose.pose.position.y = transform_stamped.transform.translation.y;
          odom.pose.pose.position.z = transform_stamped.transform.translation.z;

          // end ----------------------------------------------------------------------------
          *previous_cloud = *current_cloud;

          icp_odom_pub.publish(odom);
          // write all of this to a text file: timestamp, tx ty tz qx qy qz qw
          // for error processing

          stamped_traj_estimate << scan_in.header.stamp.toNSec() << " " << transform_stamped.transform.translation.x << " "
                                << transform_stamped.transform.translation.y << " " << transform_stamped.transform.translation.z << " "
                                << transform_stamped.transform.rotation.x  << " " << transform_stamped.transform.rotation.y << " "
                                << transform_stamped.transform.rotation.z << " " << transform_stamped.transform.rotation.w
                                << "\n";

          stamped_groundtruth << scan_in.header.stamp.toNSec() << " " << last_odom.pose.pose.position.x << " "
                              << last_odom.pose.pose.position.y << " " << last_odom.pose.pose.position.z << " "
                              << last_odom.pose.pose.orientation.x << " " << last_odom.pose.pose.orientation.y << " "
                              << last_odom.pose.pose.orientation.z << " " << last_odom.pose.pose.orientation.w
                              << "\n";
     }
};

int main(int argc, char **argv)
{

     ros::init(argc, argv, "laser_2_pointcloud");
     ros::NodeHandle node;

     std::string base_tf;
     std::string laser_topic;

     node.param("/laser_to_pointcloud/base_tf", base_tf, std::string("/base_footprint"));
     node.param("/laser_to_pointcloud/laser_topic", laser_topic, std::string("/scan"));

     LaserScanReceiver laser(node, base_tf, laser_topic);

     while (ros::ok())
     {
          ros::spinOnce();
     }

     laser.stamped_traj_estimate.close();
     laser.stamped_groundtruth.close();
}