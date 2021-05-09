#include "perception.hpp"


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>
#include <landmarks_msg/Landmarks_Msg.h>
#include <landmarks_msg/Landmark_Msg.h>

#include <visualization_msgs/MarkerArray.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>



typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

LandmarkPerception::LandmarkPerception(ros::NodeHandle& nh_):
    nh(nh_),
    it(nh),
    color_analysis(nh),
    point_cloud_synch(nh, "/realsense/depth/points", 10),
    raw_img_synch(nh, "/realsense/color/image_raw", 10),
    sync(MySyncPolicy(10), raw_img_synch, point_cloud_synch) {

        nh.param<float>("downsample_leaf_size", downsample_leaf_size, 0.01);
        nh.param<int>("min_cluster_size", min_cluster_size, 100);
        nh.param<int>("max_cluster_size", max_cluster_size, 10000);
        nh.param<double>("cluster_tolerance_factor", cluster_tolerance_factor, 2.0);
        nh.param<double>("cluster_min_z_threshold", cluster_min_z_threshold, 0.05);
        nh.param<double>("z_min", z_min, 0.01);
        nh.param<double>("z_max", z_max, 5);
        nh.param<double>("plane_segmentation_distance_threshold", plane_segmentation_distance_threshold, 0.01);
        nh.param<double>("max_cluster_area", max_cluster_area, 200);
        nh.param<double>("std_dev_landmark_y", std_dev_landmark_y, 0.05);
        nh.param<double>("std_dev_landmark_x", std_dev_landmark_x, 0.05);

        
        ROS_INFO_STREAM("Waiting for camera info pointer");

        sensor_msgs::CameraInfoConstPtr info_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/realsense/color/camera_info");
        CameraParamsPtr camera_params = std::make_shared<CameraParams>(*info_ptr);
        ROS_INFO_STREAM("Gotten camera info pointer");

        filter_point_cloud = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
        marker_pub = nh.advertise<visualization_msgs::MarkerArray>("cluster_markers", 10);
        landmark_pub = nh.advertise<landmarks_msg::Landmarks_Msg>("landmarks", 10);
        image_pub = it.advertise("rgb_image", 10);

        camera_ptr = std::make_shared<Camera>(camera_params);

        sync.registerCallback(boost::bind(&LandmarkPerception::image_callback, this, _1, _2));

    }

pcl::PointXYZ LandmarkPerception::calculate_centroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::CentroidPoint<pcl::PointXYZ> centroid_point;
    for (auto& point : *cloud) {
        centroid_point.add(point);
    }
    pcl::PointXYZ centroid;
    centroid_point.get<pcl::PointXYZ>(centroid);
    return centroid;
}

std::vector<Cluster> LandmarkPerception::get_clusters(bool publish_on, pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_xyz_pc) {
    
    //filter z cloud
    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(pcl_xyz_pc);
    pass.setFilterFieldName("z");
    pass.setFilterLimits (z_min, z_max);
    pass.filter(*pcl_xyz_pc);


    pass.setFilterFieldName("x");
    pass.setFilterLimits (-2.0, 2.0);
    pass.filter(*pcl_xyz_pc);


    ROS_INFO_STREAM("Filtering with Plane Segmentation");
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> plane_seg; // Create the segmentation object
    plane_seg.setOptimizeCoefficients (true); // Optional
    plane_seg.setModelType(pcl::SACMODEL_PLANE); // Mandatory
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(plane_segmentation_distance_threshold);
    plane_seg.setInputCloud(pcl_xyz_pc);
    plane_seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
    extract_indices.setInputCloud(pcl_xyz_pc);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(true); // we want objects, not plane
    extract_indices.filter(*pcl_xyz_pc);



    // ROS_INFO_STREAM("Scene XYZ cloud has " << pcl_xyz_pc->size() << " points");
    pcl_xyz_pc = downsample_pc<pcl::PointXYZ>(pcl_xyz_pc, downsample_leaf_size);
    // ROS_INFO_STREAM("Scene XYZ cloud has " << pcl_xyz_pc->size() << " points after downsample");

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pcl_xyz_pc, *pcl_xyz_pc, indices);

    // Calculate KD tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());
    kd_tree->setInputCloud(pcl_xyz_pc);


    // Extract indices of each cluster
    std::vector<pcl::PointIndices> cluster_indices_array;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_factor*downsample_leaf_size);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(kd_tree);
    ec.setInputCloud(pcl_xyz_pc);
    ec.extract(cluster_indices_array);
    ROS_INFO_STREAM("Found " << cluster_indices_array.size() << " clusters (unfiltered)");
    // Return vector
    std::vector<Cluster> clusters;

    // Iterate over clusters
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_scene_pcl_pc(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& cluster_indices : cluster_indices_array) {
        // Extract cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointIndices::Ptr cluster_indices_ptr(new pcl::PointIndices(cluster_indices));
        pcl::ExtractIndices<pcl::PointXYZ> extract_indices;
        extract_indices.setInputCloud(pcl_xyz_pc);
        extract_indices.setIndices(cluster_indices_ptr);
        extract_indices.setNegative(false); // we want indices, not inverse
        extract_indices.filter(*cluster);

        // Extract centroid
        pcl::PointXYZ centroid = calculate_centroid(cluster);

        // Get centroid min-max
        pcl::PointXYZ min_point, max_point;
        pcl::getMinMax3D<pcl::PointXYZ>(*cluster, min_point, max_point);

        Cluster cluster_obj(centroid, cluster);

        // Skip if the area too large (eg wall)
        if (cluster_obj.volume() > max_cluster_area) {
            ROS_INFO_STREAM("Skipping cluster, volume (" << cluster_obj.volume() << ") above threshold ("
                << max_cluster_area << ")");
            continue;
        }

        // if (cluster_obj.x_size > cluster_obj.z_size/1.2 || cluster_obj.y_size > cluster_obj.z_size/1.2) {
        //     ROS_INFO_STREAM("Not rectangular enough");
        //     continue;
        // }

        // Add cluster
        // clusters.emplace_back(centroid, cluster);
        clusters.push_back(cluster_obj);
        clusters.back().print();
        publish_pcl_cloud<pcl::PointXYZ>(clusters.back().cloud, filter_point_cloud, camera_ptr->getCameraLink());
    }

    ROS_INFO_STREAM("Number of cluster: " << clusters.size());

    // Publish clustered cloud
    if (publish_on) {
        publish_pcl_cloud<pcl::PointXYZ>(pcl_xyz_pc, filter_point_cloud, camera_ptr->getCameraLink());
    }

    return clusters;
}

 pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr LandmarkPerception::estimateCurvature(const Cluster& cluster) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(cluster.cloud);


    // Compute the normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud (cloud_in);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimation.setSearchMethod (tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::Normal>);

    normal_estimation.setRadiusSearch (0.1);

    normal_estimation.compute (*cloud_with_normals);

    // Setup the principal curvatures computation
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principal_curvatures_estimation;

    // Provide the original point cloud (without normals)
    principal_curvatures_estimation.setInputCloud (cloud_in);

    // Provide the point cloud with normals
    principal_curvatures_estimation.setInputNormals (cloud_with_normals);

    // Use the same KdTree from the normal estimation
    principal_curvatures_estimation.setSearchMethod (tree);
    principal_curvatures_estimation.setRadiusSearch (0.1);

    // Actually compute the principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principal_curvatures_estimation.compute (*principal_curvatures);

    std::cout << "output points.size (): " << principal_curvatures->points.size() << std::endl;

    // Display and retrieve the shape context descriptor vector for the 0th point.
    // pcl::PrincipalCurvatures descriptor = principal_curvatures->points[0];
    // for(pcl::PrincipalCurvatures& descriptor : principal_curvatures->points) {
    //     std::cout << descriptor << std::endl;
    // }

    // return principal_curvatures;
    return nullptr;

}

std::string LandmarkPerception::extract_colour(cv::Point2d& image_point, cv::Mat& rgb_image) {
    // const int v = static_cast<int>(image_point.x);
    // const int u = static_cast<int>(image_point.y);
    // cv::Vec3b rgb_vec = rgb_image.at<cv::Vec3b>(u,v);

    // ROS_INFO_STREAM(rgb_vec);
    // return "";
}


// # Purple: 0, Orange: 1, Yellow: 2, Blue: 3,
        // # Green: 4, Red: 5, Black: 6, Turquoise: 7
int LandmarkPerception::colour_to_id(const std::string& color) {

    if(color == "purple") {
        return 0;
    }
    else if (color == "orange") {
        return 1;
    }
    else if(color == "yellow") {
        return 2;
    }
    else if(color == "dark_blue") {
        return 3;
    }
    else if(color == "green") {
        return 4;
    }
    else if(color == "red") {
        return 5;
    }
    else if(color == "black") {
        return 6;
    }
    else if(color == "light_blue") {
        return 7;
    }
    else {
        return -1;
    }
}




void LandmarkPerception::image_callback(ImageConst& raw_image_msg, PointCloud2Ptr& point_cloud_msg) {

    //convert messages
    PointCloudPtr point_cloud(new PointCloud());
    pcl::fromROSMsg(*point_cloud_msg, *point_cloud);

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(raw_image_msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cv_ptr->image;
    cv::Mat draw_image;
    image.copyTo(draw_image);
    cv::Mat grey, image_bw;

    std::vector<Cluster> clusters = get_clusters(true,point_cloud);

    visualization_msgs::MarkerArray marker_array;
    landmarks_msg::Landmarks_Msg landmarks;

    //map of labels to number of occurances of clasification
    //will be used to increase variance if multiple clusters are labelled the same
    std::map<int, int> label_count;
    

    int id = 0;
    for(const Cluster& cluster : clusters) {

        // estimateCurvature(cluster);

        cv::Point3d pc_centroid;
        pc_centroid.x = cluster.centroid.x;
        pc_centroid.y = cluster.centroid.y;
        pc_centroid.z = cluster.centroid.z;

        cv::Point2d image_point = camera_ptr->unprojectPoints(pc_centroid);
        std::string color = color_analysis.getBestColor(image_point, image);

        char text[100];
        sprintf(text, "%s", color.c_str());
        cv::putText(image, text, cv::Point(image_point.x, image_point.y), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar::all(255), 1);

        cv::circle(draw_image,
            image_point,
            5,
            cv::Scalar( 0, 0, 255 ),
            cv::FILLED,
            cv::LINE_8);
            
        int label = colour_to_id(color);
        
        if (label_count.find(label) == label_count.end() ) {
            label_count.insert({label, 1});
        } 
        else {
            label_count[label]++;
        }


        landmarks_msg::Landmark_Msg l;
        l.label = label;
        l.x = pc_centroid.x;
        l.y = pc_centroid.y;

        landmarks.landmarks.push_back(l);

        //dont set variance yet

        visualization_msgs::Marker marker;
        marker.header.frame_id = point_cloud_msg->header.frame_id;
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = cluster.centroid.x;
        marker.pose.position.y = cluster.centroid.y;
        marker.pose.position.z = cluster.centroid.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        //only if using a MESH_RESOURCE marker type:
        id++;
        marker_array.markers.push_back(marker);
    }


    //now we know the number of possible overlaps so we can set variance
    for(landmarks_msg::Landmark_Msg& l :  landmarks.landmarks) {
        int label = l.label;
        int occurances = label_count[label];

        float s_landmark_x = std_dev_landmark_x * l.x * occurances;
        float s_landmark_y = std_dev_landmark_y * l.y * occurances;

        l.s_x = s_landmark_x;
        l.s_y = s_landmark_y;
    }

    marker_pub.publish(marker_array);
    landmark_pub.publish(landmarks);

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(raw_image_msg->header, "rgb8", image).toImageMsg();
    image_pub.publish(img_msg);


}


ColourDifferentiator::ColourDifferentiator(ros::NodeHandle& nh_)
    :   nh(nh_),
        it(nh) {

    nh.param<int>("pixel_crop", pixel_crop, 50);


    for(std::string& color : colours) {
        std::string min_threshold_param = color + "/min_threshold";
        std::string max_threshold_param = color + "/max_threshold";

        ColorThresholder color_thresholder;
        nh.getParam(min_threshold_param, color_thresholder.min_threshold);
        nh.getParam(max_threshold_param, color_thresholder.max_threshold);
        color_thresholder.color = color;
        color_thresholder.mask_pub = it.advertise(color + "/mask", 10);
        ROS_INFO_STREAM("Made mask pub: " << color + "/mask");

        thresholders.push_back(color_thresholder);
    }

}


std::string ColourDifferentiator::getBestColor(cv::Point2d& image_point, cv::Mat& rgb_image) {
    const int v = static_cast<int>(image_point.x);
    const int u = static_cast<int>(image_point.y);

    // //first crop image so that we only look at the area around the centroid
    // int start_x = std::max(v - pixel_crop, 0);
    // int end_x = std::min(v + pixel_crop, rgb_image.cols);


    // // int start_y = std::max(u - pixel_crop, 0);
    // // int end_y = std::min(u + pixel_crop, rgb_image.rows);
    // int start_y = 0;
    // int end_y = rgb_image.rows;

    // cv::Mat image_rgb_crop;
    // cv::Mat crop = rgb_image.rowRange(start_y, end_y).colRange(start_x, end_x);
    // crop.copyTo(image_rgb_crop);


    std::string best_color = "bad";
    double largest_contour = -1;

    //for now lets just mask and see how it goes
    for(ColorThresholder& thresholder: thresholders) {
        std::shared_ptr<cv::Rect2d> rect = thresholder.thresholdImage(rgb_image, u, v );

        // if (thresholder.color == "red") {
        //     // cv::rectangle(rgb_image, *rect, cv::Scalar(255,0,0), 2);

        // }

        if (rect != nullptr && rect->area() > largest_contour) {
            largest_contour = rect->area();
            best_color = thresholder.color;
            ROS_INFO_STREAM("best colour is " << best_color);
        }

    }

    return best_color;

}


std::shared_ptr<cv::Rect2d> ColorThresholder::thresholdImage(cv::Mat& rgb, int u, int v) {
    cv::Mat mask;
    cv::Point2d image_point;
    image_point.x = v;
    image_point.y = u;


    cv::inRange(rgb, cv::Scalar(min_threshold[2], min_threshold[1], min_threshold[0]),
                cv::Scalar(max_threshold[2], max_threshold[1], max_threshold[0]), mask);

    
    //close small holes
    cv::Mat morph_kernel_open = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(22,22), cv::Point(-1,-1));
    cv::Mat morph_kernel_close = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(13, 13), cv::Point(-1,-1));


    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel_open);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morph_kernel_close);



    std_msgs::Header header = std_msgs::Header();
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", mask).toImageMsg();
    mask_pub.publish(img_msg);

    //now get blobs
    cv::Mat canny_output;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    //canny detect edges
    cv::Canny(mask, canny_output, 50, 150, 3);

    //find contours
    cv::findContours(canny_output, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
    std::vector<cv::Moments> moments(contours.size());

    std::shared_ptr<cv::Rect2d> largest_rect;
    for (size_t i = 0; i < contours.size(); i++) {

        // double area = cv::contourArea(contours[i]);
        cv::Moments moment = cv::moments(contours[i]);

        cv::Point2f centroid = cv::Point2f(static_cast<float>(moment.m10/ (moment.m00 + 1e-5)),
                                    static_cast<float>(moment.m01/ (moment.m00 + 1e-5)));

        // cv::circle(rgb, centroid, 15, cv::Scalar(0,255,0));
        cv::RotatedRect rotated_rec = cv::minAreaRect(contours[i]);
        cv::Rect2d rect = rotated_rec.boundingRect();

        if (rect.contains(image_point)) {
            if (largest_rect == nullptr) {
                largest_rect = std::make_shared<cv::Rect2d>(rect);
            }
            else if (rect.area() > largest_rect->area()){
                largest_rect = std::make_shared<cv::Rect2d>(rect);
            }
        }

        // if (largest_rect == nullptr) {
        //     largest_rect = std::make_shared<cv::Rect2d>(rect);
        // }
        // else if (rect.area() > largest_rect->area()){
        //     largest_rect = std::make_shared<cv::Rect2d>(rect);
        // }
        


        // moments[i] = cv::moments(contours[i]);

        //     //add 1e-5 tp avoid division by zero
        // cv::Point2f centroid = cv::Point2f(static_cast<float>(moments[i].m10/ (moments[i].m00 + 1e-5)),
        //                             static_cast<float>(moments[i].m01/ (moments[i].m00 + 1e-5)));

        // if (std::abs(centroid.x - v) < 20 && std::abs(centroid.y - u) < 20) {
        //     largest_area = area;
        //     cv::circle(rgb, centroid, 15, cv::Scalar(0,255,0));
        // }




    }
    
    if (largest_rect == nullptr) {
        return std::make_shared<cv::Rect2d>();
    }
    else {
        return largest_rect;
    }
}