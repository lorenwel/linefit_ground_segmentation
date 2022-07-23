#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef boost::shared_ptr<PointCloud> PointCloudPtr;
typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;
