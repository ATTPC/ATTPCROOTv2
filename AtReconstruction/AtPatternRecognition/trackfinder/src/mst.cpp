#include "mst.h"

#include <numeric>

namespace mst
{
    std::vector<edge> calculateSquareDistances(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, MstMetric metric)
    {
        const size_t cloud_size = cloud->size();
        std::vector<edge> result;
        result.reserve(cloud_size * (cloud_size - 1) / 2);

        for(size_t i = 0; i < cloud_size; ++i)
        {
            pcl::PointXYZI pointA = (*cloud)[i];

            for(size_t j = i + 1; j < cloud_size; ++j)
            {
                pcl::PointXYZI pointB = (*cloud)[j];

                float distance = metric(pointA, pointB, i, j, cloud);

                edge e = {
                    i, // voxelIndexA
                    j, // voxelIndexB
                    distance // distance
                };

                result.push_back(e);
            }
        }

        std::sort(result.begin(), result.end());

        return result;
    }

    std::vector<state> calculateMinimumSpanningTree(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, MstMetric metric, MstFilter filter)
    {
        std::vector<edge> edges = calculateSquareDistances(cloud, metric);
        std::vector<state> result;

        std::vector<edge> selectedEdges;
        std::vector<size_t> groups(cloud->size());
        for(size_t i = 0; i < groups.size(); i++) {
          groups[i] = i;
        }

        for(std::vector<edge>::const_iterator edge = edges.begin(); edge != edges.end(); ++edge)
        {
            size_t groupA = groups[edge->voxelIndexA];
            size_t groupB = groups[edge->voxelIndexB];

            // if no circle
            if(groupA != groupB && filter(cloud, *edge, selectedEdges, groups))
            {
                selectedEdges.push_back(*edge);

                // merge groups
                for(std::vector<size_t>::iterator it = groups.begin(); it != groups.end(); ++it)
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
}
