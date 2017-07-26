#ifndef ATTRAJECTORY_HH
#define ATTRAJECTORY_HH

#include <vector>
#include <Eigen/Core>

#include "ATHit.hh"

class ATTrajectory
{
protected:
    std::vector<ATHit> _hits;
    float _approximateTrajectoryLength = 0.0f;
    float _averageCurvature = 0.0f;
    Eigen::Vector3f _centroidPoint;
    Eigen::Vector3f _mainDirection;

public:
    ATTrajectory();
    ATTrajectory(std::vector<ATHit> hits, float approximateTrajectoryLength, float averageCurvature, Eigen::Vector3f centroidPoint, Eigen::Vector3f mainDirection);

    std::vector<ATHit> const &GetHits() const;
    float const &GetApproximateTrajectoryLength() const;
    float const &GetAverageCurvature() const;
    Eigen::Vector3f const &GetCentroidPoint() const;
    Eigen::Vector3f const &GetMainDirection() const;
};

#endif
