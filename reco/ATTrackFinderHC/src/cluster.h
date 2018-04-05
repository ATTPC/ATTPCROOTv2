#pragma once

#include <vector>
#include <pcl/io/io.h>

class Cluster
{
protected:
    size_t pointIndexCount;
    std::vector<pcl::PointIndicesPtr> clusters;
    mutable Eigen::ArrayXXi relationshipMatrix;

    void calculateRelationshipMatrixIfNecessary() const;

public:
    Cluster();
    Cluster(std::string const &filepath);
    Cluster(std::vector<pcl::PointIndicesPtr> const &clusters, size_t pointIndexCount);

    void save(std::string filepath, std::string comment = "") const;
    void load(std::string filepath);

    std::vector<pcl::PointIndicesPtr> const &getClusters() const;
    size_t getPointIndexCount() const;

    int operator-(Cluster const &rhs) const;
};
