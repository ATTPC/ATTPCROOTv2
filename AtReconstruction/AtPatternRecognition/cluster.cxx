#include "cluster.h"

#include <boost/algorithm/string/replace.hpp>

#include <cctype>
#include <fstream>
#include <limits>

Cluster::Cluster(std::vector<pcl::PointIndicesPtr> const &clustersIn, size_t pointIndexCountIn)
{
   clusters = clustersIn;
   pointIndexCount = pointIndexCountIn;
}

std::vector<pcl::PointIndicesPtr> const &Cluster::getClusters() const
{
   return this->clusters;
}

size_t Cluster::getPointIndexCount() const
{
   return this->pointIndexCount;
}
