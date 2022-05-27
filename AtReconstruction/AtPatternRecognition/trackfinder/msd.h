#ifndef MSD_H
#define MSD_H

#include <pcl/point_cloud.h> // for PointCloud, PointCloud<>::Ptr
#include <pcl/point_types.h> // for PointXYZ

namespace msd {
float first_quartile(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
}

#endif
