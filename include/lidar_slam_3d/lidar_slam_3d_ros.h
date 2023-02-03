#ifndef LIDAR_SLAM_3D_ROS_H
#define LIDAR_SLAM_3D_ROS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include "map_builder.h"
#include "floor_filter.h"
#include <pcl_ros/point_cloud.h>       // TODO @Yimin Zhao
#include <pcl/point_types.h>           // TODO @Yimin Zhao
#include <pcl/filters/passthrough.h>   // TODO @Yimin Zhao
#include <pcl/filters/statistical_outlier_removal.h>  // TODO @Yimin Zhao

#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::stringstream
#include <cmath>

typedef Eigen::Matrix<float, 6, 1> Vector6f;

class LidarSlam3dRos
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LidarSlam3dRos();
    ~LidarSlam3dRos() {}

private:
    Vector6f getPose(const Eigen::Matrix4f& T);
    void publishLoop();
    void publishMap();
    void publishConstraintList();
    void publishPose(const Vector6f& pose, const ros::Time& t);
    void publishPath();
    void publishTf(const Vector6f& pose, const ros::Time& t);
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    bool optimizationCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);   
    void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered);

private:
    std::shared_ptr<std::thread> publish_thread_;
    ros::Publisher map_pub_;
    ros::Publisher path_pub_;
    ros::Publisher gps_path_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher filtered_point_cloud_pub_;
    ros::Publisher filtered_points_pub_; // TODO @Yimin Zhao    
    ros::Publisher floor_points_pub_;
    ros::Publisher constraint_list_pub_;      
    ros::ServiceServer optimization_srv_;
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_; // add imu_sub_
    tf::TransformBroadcaster tf_broadcaster_;
    nav_msgs::Path path_msg_;
    nav_msgs::Path gps_path_msg_;

    boost::optional<Eigen::Vector3d> gps_origin_;

    std::string base_frame_;
    std::string map_frame_;
    std::string imu_frame_;
    ros::Time initial_ros_time_;  // TODO @Yimin Zhao
    ros::Duration time_stamp_;    // TODO @Yimin Zhao
    double time_stamp;            // TODO @Yimin Zhao
    double publish_freq_;
    double min_scan_distance_;
    double speed_stage_rpm_;      // TODO @Yimin Zhao
    double angle_velocity_;       // TODO @Yimin Zhao
    double angley;                // TODO @Yimin Zhao
    double angle_scale_;          // TODO @Yimin Zhao
    
    bool enable_floor_filter_;
    bool initial_point_cloud_;

    lidar_slam_3d::FloorFilter floor_filter_;
    lidar_slam_3d::MapBuilder map_builder_;
};

#endif // LIDAR_SLAM_3D_ROS_H
