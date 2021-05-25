#ifndef AtHIERARCHICALCLUSTERINGSMOOTHENCLOUD_HH
#define AtHIERARCHICALCLUSTERINGSMOOTHENCLOUD_HH

#include <pcl/io/io.h>

namespace AtHierarchicalClusteringSmoothenCloud {
pcl::PointCloud<pcl::PointXYZ>::Ptr
SmoothenCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k, bool useMedian = false);
pcl::PointCloud<pcl::PointXYZI>::Ptr
SmoothenCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double radius, bool useMedian = false);
} // namespace AtHierarchicalClusteringSmoothenCloud

#endif
