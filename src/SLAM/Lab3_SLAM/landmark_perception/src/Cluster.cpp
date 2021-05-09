//
// Created by williamtalbot on 18/9/20.
//

#include "Cluster.hpp"
#include <pcl/common/common.h>
#include "perception.hpp"

#include <ros/ros.h>
#include <iomanip>
#include <sstream>

//#define PRINTING_ON

Cluster::Cluster(const pcl::PointXYZ& centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
:   centroid(centroid),
    cloud(cloud),
    colour("none"),
    label(-1)
{
    // Get centroid min-max
    pcl::getMinMax3D<pcl::PointXYZ>(*cloud, min_point, max_point);
    x_size = max_point.x - min_point.x;
    y_size = max_point.y - min_point.y;
    z_size = max_point.z - min_point.z;
}


double Cluster::volume() const {
    return x_size*y_size*z_size;
}

void Cluster::set_name(const std::string& name) {
    this->colour = name;
}

void Cluster::set_colour(const std::string& colour) {
    this->colour = colour;
}

void Cluster::print() const {
    ROS_INFO_STREAM("==================== Cluster ====================");
    ROS_INFO_STREAM("\t            Label: " << (label == -1 ? "[unassigned]" : std::to_string(label)));
    ROS_INFO_STREAM("\tNumber of Points: " << cloud->size());
    ROS_INFO_STREAM("\t        Centroid: (" << std::setprecision(3)
        << centroid.x << ", " << centroid.y << ", " << centroid.z << ")");
    ROS_INFO_STREAM("\t          Volume: " << volume());
    ROS_INFO_STREAM("\t  Geometric Size: (" << x_size << ", " << y_size << ", " << z_size << ")");
    ROS_INFO_STREAM("=================================================");
}