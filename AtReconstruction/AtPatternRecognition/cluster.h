#ifndef CLUSTER_H
#define CLUSTER_H

#include <pcl/io/io.h>
#include <vector>

class Cluster {
protected:
   size_t pointIndexCount;
   std::vector<pcl::PointIndicesPtr> clusters;

public:
   Cluster(){};
   Cluster(std::vector<pcl::PointIndicesPtr> const &clusters, size_t pointIndexCount);

   std::vector<pcl::PointIndicesPtr> const &getClusters() const;
   size_t getPointIndexCount() const;
};

#endif
