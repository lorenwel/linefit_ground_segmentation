#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

typedef std::pair<pcl::PointXYZ, pcl::PointXYZ> PointLine;
