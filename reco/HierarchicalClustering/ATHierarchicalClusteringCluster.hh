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

    void CalculateRelationshipMatrixIfNecessary() const;

public:
    ATHierarchicalClusteringCluster();
    ATHierarchicalClusteringCluster(std::string const &filepath);
    ATHierarchicalClusteringCluster(std::vector<std::vector<size_t>> const &clusters, size_t pointIndexCount);

    void Save(std::string filepath, std::string comment = "") const;
    void Load(std::string filepath);

    std::vector<std::vector<size_t>> const &GetClusters() const;
    size_t GetPointIndexCount() const;

    int operator-(ATHierarchicalClusteringCluster const &rhs) const;
};

#endif
