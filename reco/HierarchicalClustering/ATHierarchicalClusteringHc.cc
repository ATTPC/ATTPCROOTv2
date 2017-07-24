#include "ATHierarchicalClusteringHc.hh"

#include <algorithm>
#include <pcl/kdtree/kdtree_flann.h>

namespace ATHierarchicalClusteringHc
{
    std::vector<triplet> generateTriplets(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, size_t nnKandidates, size_t nBest, float maxError)
    {
        std::vector<triplet> triplets;
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(cloud);

        for(size_t pointIndexB = 0; pointIndexB < cloud->size(); ++pointIndexB)
        {
            auto const &pointB = (*cloud)[pointIndexB];
            Eigen::Vector3f pointBEigen(pointB.x, pointB.y, pointB.z);

            std::vector<triplet> tripletCandidates;

            std::vector<int> nnIndices;
            nnIndices.reserve(nnKandidates);
            std::vector<float> nnbSquaredDistances;
            nnbSquaredDistances.reserve(nnKandidates);
            int const nnFound = kdtree.nearestKSearch(*cloud, (int)pointIndexB, (int)nnKandidates, nnIndices, nnbSquaredDistances);

            for(size_t pointIndexIndexA = 0; pointIndexIndexA < nnFound; ++pointIndexIndexA)
            {
                size_t const pointIndexA = nnIndices[pointIndexIndexA];
                auto const &pointA = (*cloud)[pointIndexA];
                Eigen::Vector3f pointAEigen(pointA.x, pointA.y, pointA.z);

                Eigen::Vector3f directionAB = pointBEigen - pointAEigen;
                directionAB /= directionAB.norm();

                for(size_t pointIndexIndexC = pointIndexIndexA + 1; pointIndexIndexC < nnFound; ++pointIndexIndexC)
                {
                    size_t const pointIndexC = nnIndices[pointIndexIndexC];
                    auto const &pointC = (*cloud)[pointIndexC];
                    Eigen::Vector3f pointCEigen(pointC.x, pointC.y, pointC.z);

                    Eigen::Vector3f directionBC = pointCEigen - pointBEigen;
                    directionBC /= directionBC.norm();

                    float const angle = directionAB.dot(directionBC);

                    // calculate error
                    float const error = -0.5f * (angle - 1.0f);

                    if(error <= maxError)
                    {
                        // calculate center
                        Eigen::Vector3f center = (pointAEigen + pointBEigen + pointCEigen) / 3.0f;

                        // calculate direction
                        Eigen::Vector3f direction = pointCEigen - pointBEigen;
                        direction /= direction.norm();

                        triplet newTriplet;

                        newTriplet.pointIndexA = pointIndexA;
                        newTriplet.pointIndexB = pointIndexB;
                        newTriplet.pointIndexC = pointIndexC;
                        newTriplet.center = center;
                        newTriplet.direction = direction;
                        newTriplet.error = error;

                        tripletCandidates.push_back(newTriplet);
                    }
                }
            }

            // order triplet candidates
            std::sort(tripletCandidates.begin(), tripletCandidates.end(), [](triplet const &lhs, triplet const &rhs) { return lhs.error < rhs.error; });

            // use the n best candidates
            for (size_t i = 0; i < std::min(nBest, tripletCandidates.size()); ++i)
            {
                triplets.push_back(tripletCandidates[i]);
            }
        }

        return triplets;
    }

    static Eigen::MatrixXf calculateDistanceMatrix(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::vector<triplet> const &triplets, TripletMetric tripletMetric)
    {
        size_t const tripletSize = triplets.size();
        Eigen::MatrixXf result(tripletSize, tripletSize);

        for (size_t i = 0; i < tripletSize; ++i)
        {
            result(i, i) = 0.0f;

            for (size_t j = i + 1; j < tripletSize; ++j)
            {
                result(i, j) = tripletMetric(triplets[i], triplets[j], cloud);
                result(j, i) = result(i, j);
            }
        }

        return result;
    }

    cluster_history calculateHc(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, std::vector<triplet> const &triplets, ClusterMetric clusterMetric, TripletMetric tripletMetric)
    {
        cluster_history result;

        result.triplets = triplets;

        // calculate distance-Matrix
        Eigen::MatrixXf distanceMatrix = calculateDistanceMatrix(cloud, result.triplets, tripletMetric);

        cluster_group currentGeneration;

        // init first generation
        for(size_t i = 0; i < result.triplets.size(); ++i)
        {
            cluster newCluster;
            newCluster.push_back(i);
            currentGeneration.clusters.push_back(newCluster);
        }

        // merge until only one cluster left
        while (currentGeneration.clusters.size() > 1)
        {
            float bestClusterDistance = std::numeric_limits<float>::infinity();
            std::pair<size_t, size_t> bestClusterPair;

            // find best cluster-pair
            for (size_t i = 0; i < currentGeneration.clusters.size(); ++i)
            {
                for (size_t j = i + 1; j < currentGeneration.clusters.size(); ++j)
                {
                    float const clusterDistance = clusterMetric(currentGeneration.clusters[i], currentGeneration.clusters[j], distanceMatrix, cloud);

                    if (clusterDistance < bestClusterDistance)
                    {
                        bestClusterDistance = clusterDistance;
                        bestClusterPair = std::pair<size_t, size_t>(i, j);
                    }
                }
            }

            // merge cluster-pair
            cluster merged(currentGeneration.clusters[bestClusterPair.first]);
            merged.insert(merged.cend(), currentGeneration.clusters[bestClusterPair.second].cbegin(), currentGeneration.clusters[bestClusterPair.second].cend());

            // set best cluster distance for current generation and add to results
            currentGeneration.bestClusterDistance = bestClusterDistance;
            result.history.push_back(currentGeneration);

            // copy data to next generation
            cluster_group nextGeneration;
            nextGeneration.clusters.reserve(currentGeneration.clusters.size() - 1);

            for (size_t i = 0; i < currentGeneration.clusters.size(); ++i)
            {
                if (i != bestClusterPair.first && i != bestClusterPair.second)
                    nextGeneration.clusters.push_back(currentGeneration.clusters[i]);
            }

            nextGeneration.clusters.push_back(merged);

            // overwrite current generation
            currentGeneration = nextGeneration;
        }

        // set last cluster distance and add to results
        currentGeneration.bestClusterDistance = std::numeric_limits<float>::infinity();
        result.history.push_back(currentGeneration);

        return result;
    }

    cluster_group getBestClusterGroup(cluster_history const &history, float bestClusterDistanceDelta)
    {
        float lastBestClusterDistance = history.history[0].bestClusterDistance;

        for (cluster_group const &clusterGroup : history.history)
        {
            float const bestClusterDistanceChange = clusterGroup.bestClusterDistance - lastBestClusterDistance;
            lastBestClusterDistance = clusterGroup.bestClusterDistance;

            if (bestClusterDistanceChange > bestClusterDistanceDelta)
                return clusterGroup;
        }

        return history.history[history.history.size() - 1];
    }

    cluster_group cleanupClusterGroup(cluster_group const &clusterGroup, size_t minTriplets)
    {
        cluster_group cleanedGroup;
        cleanedGroup.bestClusterDistance = clusterGroup.bestClusterDistance;
        cleanedGroup.clusters.resize(clusterGroup.clusters.size());

        auto newEnd = std::copy_if(clusterGroup.clusters.cbegin(), clusterGroup.clusters.cend(), cleanedGroup.clusters.begin(), [&](cluster const &clusterEl) {
            return clusterEl.size() >= minTriplets;
        });
        cleanedGroup.clusters.resize(std::distance(cleanedGroup.clusters.begin(), newEnd));

        return cleanedGroup;
    }

    ATHierarchicalClusteringCluster toCluster(std::vector<triplet> const &triplets, cluster_group const &clusterGroup, size_t pointIndexCount)
    {
        std::vector<std::vector<size_t>> result;

        for (auto const &currentCluster : clusterGroup.clusters)
        {
            std::vector<size_t> pointIndices;

            // add point indices
            for (auto const &currentTripletIndex : currentCluster)
            {
                triplet const &currentTriplet = triplets[currentTripletIndex];

                pointIndices.push_back(currentTriplet.pointIndexA);
                pointIndices.push_back(currentTriplet.pointIndexB);
                pointIndices.push_back(currentTriplet.pointIndexC);
            }

            // sort point-indices and remove duplikates
            std::sort(pointIndices.begin(), pointIndices.end());
            auto newEnd = std::unique(pointIndices.begin(), pointIndices.end());
            pointIndices.resize(std::distance(pointIndices.begin(), newEnd));

            result.push_back(pointIndices);
        }

        return ATHierarchicalClusteringCluster(result, pointIndexCount);
    }
}
