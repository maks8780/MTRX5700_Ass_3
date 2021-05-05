// This file uses Iterative Closest Point algorithm to determine if one 
// pointCloud is just a rigid transformation of another by minimizing the distances 
// between the points of two pointclouds and rigidly transforming them.

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/LaserScan.h>
#include "icp_data_loader.hpp"


int main (int argc, char** argv) {
	ros::init(argc, argv, "icp_registration");

	ros::NodeHandle n("~");
	ros::Publisher pcl_pub1 = n.advertise<sensor_msgs::PointCloud2>("icp_registration/points/filtered1", 1);
	ros::Publisher pcl_pub2 = n.advertise<sensor_msgs::PointCloud2>("icp_registration/points/filtered2", 1);

	IcpDataLoader loader("/home/mtrx5700/Desktop/MTRX5700/Assignment_2/data/icp_pose_1.bag");
	std::shared_ptr<IcpData> data = loader.load_data();

	IcpDataLoader loader1("/home/mtrx5700/Desktop/MTRX5700/Assignment_2/data/icp_pose_2.bag");
	std::shared_ptr<IcpData> data1 = loader1.load_data();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in = data->point_cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out = data1->point_cloud;
	
	// remove NaN points from both point clouds
	std::vector<int> indices;
  	pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
	pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, indices);

	std::cout << "Loaded " << cloud_in->size () << " data points to cloud 1" << std::endl;
	std::cout << "Loaded " << cloud_out->size () << " data points to cloud 2" << std::endl;

	// filter field of vision to keep only the feature points (the brick)
	cloud_in = filter_cloud_z<pcl::PointXYZ>(cloud_in, 0.1, 0.9);
	cloud_out = filter_cloud_z<pcl::PointXYZ>(cloud_out, 0.1, 0.9);

	// down sample for speed
	cloud_in = downsample_pc<pcl::PointXYZ>(cloud_in, 0.01f);
	cloud_out = downsample_pc<pcl::PointXYZ>(cloud_out, 0.01f);

	std::cout << "After downsampling  " << cloud_in->size () << " data points to input:" << std::endl;

	// perform simple rigid transform on the pointcloud 
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	// define input and output point clouds
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);

	// Set the max correspondence distance 5cm
	icp.setMaxCorrespondenceDistance (0.5f);

	// Set the transformation epsilon
	icp.setTransformationEpsilon (1e-5);

	// Find iterative closest point
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;

	std::cout << icp.getFinalTransformation() << std::endl;

	return (0);
}