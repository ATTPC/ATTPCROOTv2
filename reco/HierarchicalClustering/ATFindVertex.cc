#include "ATFindVertex.hh"

#include <iostream>

Eigen::Vector3f ATFindVertex(std::vector<ATTrajectory> const &trajectories)
{
    std::vector<float> bestSplinePositions(trajectories.size());
    std::vector<Eigen::Vector3f> bestSplinePoints(trajectories.size());
    std::vector<float> bestDistanceSums(trajectories.size());

    auto calculateBestDistanceSum = [&](size_t trajectoryIndex, Eigen::Vector3f const &refPoint)
    {
        float result = 0.0f;

        for (size_t i = 0; i < trajectories.size(); ++i)
        {
            if (i != trajectoryIndex)
                result += (refPoint - bestSplinePoints[i]).norm();
        }

        return result;
    };

    // init bestSplinePoints
    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        ATCubicSplineFit const &splineFit = trajectories[i].GetCubicSplineFit();
        bestSplinePoints[i] = splineFit.CalculatePoint(bestSplinePositions[i]);
    }

    // init bestDistanceSums
    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        bestDistanceSums[i] = calculateBestDistanceSum(i, bestSplinePoints[i]);
    }

    // greedy search for best positions
    // optimize one trajectory-position after the other
    bool somethingChanged;

    do
    {
        somethingChanged = false;

        for (size_t i = 0; i < trajectories.size(); ++i)
        {
            ATCubicSplineFit const &splineFit = trajectories[i].GetCubicSplineFit();

            bool const direction =
                calculateBestDistanceSum(i, splineFit.CalculatePoint(bestSplinePositions[i] + 0.01f)) <
                calculateBestDistanceSum(i, splineFit.CalculatePoint(bestSplinePositions[i] - 0.01f));

            for (float offset = 10000.0f; offset >= 0.01f; offset /= 2.0f)
            {
                float const bestSplinePositionCandidate = bestSplinePositions[i] + (offset * (direction ? 1.0f : -1.0f));
                Eigen::Vector3f const bestSplinePointCandidate = splineFit.CalculatePoint(bestSplinePositionCandidate);

                float const newBestDistanceSum = calculateBestDistanceSum(i, bestSplinePointCandidate);

                if (newBestDistanceSum < bestDistanceSums[i])
                {
                    bestDistanceSums[i] = newBestDistanceSum;
                    bestSplinePositions[i] = bestSplinePositionCandidate;
                    bestSplinePoints[i] = bestSplinePointCandidate;
                    somethingChanged = true;
                }
            }
        }
    }
    while(somethingChanged);

    Eigen::Vector3f resultPosition(0.0f, 0.0f, 0.0f);

    for (Eigen::Vector3f const &bestSplinePoint : bestSplinePoints)
    {
        resultPosition += bestSplinePoint;
    }

    resultPosition /= static_cast<float>(bestSplinePoints.size());

    return resultPosition;
}
