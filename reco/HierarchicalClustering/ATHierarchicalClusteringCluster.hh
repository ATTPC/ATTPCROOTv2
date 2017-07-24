#ifndef ATHIERARCHICALCLUSTERINGCLUSTER_HH
#define ATHIERARCHICALCLUSTERINGCLUSTER_HH

#include <vector>

#include <Eigen/Core>

class ATHierarchicalClusteringCluster
{
protected:
    size_t _pointIndexCount;
    std::vector<std::vector<size_t>> _clusters;
    mutable Eigen::ArrayXXi _relationshipMatrix;

    void calculateRelationshipMatrixIfNecessary() const;

public:
    ATHierarchicalClusteringCluster();
    ATHierarchicalClusteringCluster(std::string const &filepath);
    ATHierarchicalClusteringCluster(std::vector<std::vector<size_t>> const &clusters, size_t pointIndexCount);

    void save(std::string filepath, std::string comment = "") const;
    void load(std::string filepath);

    std::vector<std::vector<size_t>> const &getClusters() const;
    size_t getPointIndexCount() const;

    int operator-(ATHierarchicalClusteringCluster const &rhs) const;
};

#endif
