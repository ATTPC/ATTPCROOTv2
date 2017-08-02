#ifndef ATTRAJECTORY_HH
#define ATTRAJECTORY_HH

#include <vector>
#include <Eigen/Core>

#include "ATCubicSplineFit.hh"
#include "ATHit.hh"

class ATTrajectory
{
protected:
    std::vector<ATHit> _hits;
    size_t _startHitIndex = 0;
    size_t _endHitIndex = 0;
    float _approximateTrajectoryLength = 0.0f;
    float _averageCurvature = 0.0f;
    Eigen::Vector3f _centroidPoint;
    Eigen::Vector3f _mainDirection;
    ATCubicSplineFit _cubicSplineFit;

public:
    ATTrajectory(
        std::vector<ATHit> const &hits,
        size_t startHitIndex,
        size_t endHitIndex,
        float approximateTrajectoryLength,
        float averageCurvature,
        Eigen::Vector3f const &centroidPoint,
        Eigen::Vector3f const &mainDirection,
        ATCubicSplineFit const &cubicSplineFit
    );

    std::vector<ATHit> const &GetHits() const;
    size_t const &GetStartHitIndex() const;
    ATHit const &GetStartHit() const;
    size_t const &GetEndHitIndex() const;
    ATHit const &GetEndHit() const;

    float const &GetApproximateTrajectoryLength() const;
    float const &GetAverageCurvature() const;
    Eigen::Vector3f const &GetCentroidPoint() const;
    Eigen::Vector3f const &GetMainDirection() const;
    ATCubicSplineFit const &GetCubicSplineFit() const;
};

#endif
