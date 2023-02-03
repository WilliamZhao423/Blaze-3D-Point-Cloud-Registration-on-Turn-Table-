#ifndef FLOOR_FILTER_H
#define FLOOR_FILTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace lidar_slam_3d
{

class FloorFilter
{
public:
    FloorFilter();
    ~FloorFilter() {}

    void filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, // TODO @Yimin Zhao
                pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,     // TODO @Yimin Zhao
                pcl::PointCloud<pcl::PointXYZ>::Ptr& floor_cloud);       // TODO @Yimin Zhao     

private:
    double height_clip_range_;
    double point_normal_threshhold_;
    double floor_normal_threshhold_;
    int floor_min_points_num_;
};

} // namespace lidar_slam_3d

#endif // FLOOR_FILTER_H
