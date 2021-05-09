#pragma once
#include <pcl/common/centroid.h>
#include <ros/ros.h>

/*
 * Simple Cluster object containing at its core a centroid and a cloud
 */
class Cluster {
public:
    // Constructor
    Cluster(const pcl::PointXYZ& centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    // Core data is public
    pcl::PointXYZ centroid;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointXYZ min_point, max_point;
    double x_size, y_size, z_size;
    int label;

    // Helper functions
    double volume() const;
    void set_name(const std::string& name);
    void set_colour(const std::string& colour);
    void print() const;

private:
    std::string colour;
};

