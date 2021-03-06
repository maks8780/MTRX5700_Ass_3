#include <ros/ros.h>
#include <nodelet/loader.h>

#include "perception.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <stdio.h>

#include <string>
#include <vector>
#include <algorithm>
#include <map>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "landmark_perception");
    ros::NodeHandle n;
    LandmarkPerception p(n);
    ros::spin();
    
}