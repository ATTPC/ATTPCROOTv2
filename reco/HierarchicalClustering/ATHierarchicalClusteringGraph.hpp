#ifndef ATHIERARCHICALCLUSTERINGGRAPH_H
#define ATHIERARCHICALCLUSTERINGGRAPH_H

#include <limits>
#include <numeric>
#include <vector>
#include <pcl/io/io.h>

namespace ATHierarchicalClusteringGraph
{
    struct edge {
        size_t voxelIndexA;
        size_t voxelIndexB;
        float distance;
    };

    struct state {
        std::vector<edge> edges;
        std::vector<size_t> groups;
    };

    template <class T>
    using MstMetric = std::function<float(T const &, T const &)>;

    template <class T>
    inline float DefaultMstMetric(T const &pointA, T const &pointB)
    {
        Eigen::Vector3f diff(
            pointA.x - pointB.x,
            pointA.y - pointB.y,
            pointA.z - pointB.z
        );

        return diff.dot(diff);
    }

    template <class T>
    std::vector<edge> CalculateEdges(pcl::PointCloud<T> const &cloud, MstMetric<T> metric)
    {
        const size_t cloud_size = cloud.size();
        std::vector<edge> result;
        result.reserve(cloud_size * (cloud_size - 1) / 2);

        for(size_t i = 0; i < cloud_size; ++i)
        {
            auto const &pointA = cloud[i];

            for(size_t j = i + 1; j < cloud_size; ++j)
            {
                auto const &pointB = cloud[j];

                float distance = metric(pointA, pointB);

                edge e = {
                    i, // voxelIndexA
                    j, // voxelIndexB
                    distance // distance
                };

                result.push_back(e);
            }
        }

        std::sort(
            result.begin(),
            result.end(),
            [] (edge const &lhs, edge const &rhs) {
                return lhs.distance < rhs.distance;
            }
        );

        return result;
    }

    template <class T>
    std::vector<state> CalculateMinimumSpanningTree(pcl::PointCloud<T> const &cloud, MstMetric<T> metric = DefaultMstMetric<T>)
    {
        std::vector<edge> edges = CalculateEdges(cloud, metric);
        std::vector<state> result;

        std::vector<edge> selectedEdges;
        std::vector<size_t> groups(cloud.size());
        std::iota(groups.begin(), groups.end(), (size_t)0);

        for(edge const &edgeElement : edges)
        {
            size_t groupA = groups[edgeElement.voxelIndexA];
            size_t groupB = groups[edgeElement.voxelIndexB];

            // if no circle
            if(groupA != groupB)
            {
                selectedEdges.push_back(edgeElement);

                // merge groups
                for(auto it = groups.begin(); it != groups.end(); ++it)
                {
                    if(*it == groupB)
                        *it = groupA;
                }

                state state = {
                    selectedEdges, // edges
                    groups, // groups
                };

                result.push_back(state);
            }
        }

        return result;
    }

    Eigen::MatrixXf CalculateAllPairsShortestPath(std::vector<edge> const &edges, size_t nodeCount)
    {
        Eigen::MatrixXf result = Eigen::MatrixXf::Constant(nodeCount, nodeCount, std::numeric_limits<float>::infinity());

        // init 
        for (edge const &edgeElement : edges)
        {
            float const distance = std::sqrt(edgeElement.distance);
            result(edgeElement.voxelIndexA, edgeElement.voxelIndexB) = distance;
            result(edgeElement.voxelIndexB, edgeElement.voxelIndexA) = distance;
        }

        // Floyd's Algorithm
        for (size_t k = 0; k < nodeCount; ++k)
        {
            for (size_t i = 0; i < (nodeCount - 1); ++i)
            {
                for (size_t j = (i + 1); j < nodeCount; ++j)
                {
                    float const newDistance = std::min(result(i, j), (result(i, k) + result(k, j)));
                    result(i, j) = newDistance;
                    result(j, i) = newDistance;
                }
            }
        }

        return result;
    }

}
#endif
