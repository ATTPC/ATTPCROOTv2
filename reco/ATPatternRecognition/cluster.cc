#include "cluster.hh"

#include <cctype>
#include <fstream>
#include <limits>

#include <boost/algorithm/string/replace.hpp>

Cluster::Cluster(std::vector<pcl::PointIndicesPtr> const &clusters,
                 size_t pointIndexCount) {
  this->clusters = clusters;
  this->pointIndexCount = pointIndexCount;
}

std::vector<pcl::PointIndicesPtr> const &Cluster::getClusters() const {
  return this->clusters;
}

size_t Cluster::getPointIndexCount() const { return this->pointIndexCount; }

