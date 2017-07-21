#ifndef ATHIERARCHICALCLUSTERINGCLUSTER_HH
#define ATHIERARCHICALCLUSTERINGCLUSTER_HH

#include <vector>
#include <pcl/io/io.h>

class ATHierarchicalClusteringCluster
{
protected:
    size_t pointIndexCount;
    std::vector<pcl::PointIndicesPtr> clusters;
    mutable Eigen::ArrayXXi relationshipMatrix;

    void calculateRelationshipMatrixIfNecessary() const;

public:
    ATHierarchicalClusteringCluster();
    ATHierarchicalClusteringCluster(std::string const &filepath);
    ATHierarchicalClusteringCluster(std::vector<pcl::PointIndicesPtr> const &clusters, size_t pointIndexCount);

    void save(std::string filepath, std::string comment = "") const;
    void load(std::string filepath);

    std::vector<pcl::PointIndicesPtr> const &getClusters() const;
    size_t getPointIndexCount() const;

    int operator-(ATHierarchicalClusteringCluster const &rhs) const;
};

#endif
