cmake_minimum_required(VERSION 2.8.3)
project(a3_icp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	COMPONENTS
    laser_geometry
 	rospy 
 	interactive_markers
    pluginlib
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    geometric_shapes
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
    image_transport
    cv_bridge
    roscpp
    std_msgs
    rosbag
    )


## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(PCL 1.2 REQUIRED)



catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES a3_icp
    CATKIN_DEPENDS 
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    geometry_msgs
    sensor_msgs
    image_transport
    cv_bridge
    roscpp
    std_msgs
    rosbag
    std_msgs sensor_msgs roslib
    laser_geometry
#  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable (icp_main 
    src/icp_main.cpp
    src/icp_data_loader.cpp)

    
target_link_libraries (icp_main 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

add_executable (quaternion_to_rotation_matrix
    src/quaternion_to_rotation_matrix.cpp)

target_link_libraries (quaternion_to_rotation_matrix
${catkin_LIBRARIES})

add_executable (get_laser_data 
    src/get_laser_data.cpp)

    
target_link_libraries (get_laser_data 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

## catkin specific configuration
