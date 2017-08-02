#ifndef ATTRAJECTORY_HH
#define ATTRAJECTORY_HH

#include <vector>
#include <Eigen/Core>

#include "ATCubicSplineFit.hh"
#include "ATHit.hh"

class ATTrajectory
{
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
    Eigen::Vector3f GetStartHitVector() const;
    size_t const &GetEndHitIndex() const;
    ATHit const &GetEndHit() const;
    Eigen::Vector3f GetEndHitVector() const;

    static float GetPositionOnMainDirection(Eigen::Vector3f const &centroidPoint, Eigen::Vector3f const &mainDirection, Eigen::Vector3f const &point);
    float GetPositionOnMainDirection(Eigen::Vector3f const &point) const;

    float const &GetApproximateTrajectoryLength() const;
    float const &GetAverageCurvature() const;
    Eigen::Vector3f const &GetCentroidPoint() const;
    Eigen::Vector3f const &GetMainDirection() const;
    ATCubicSplineFit const &GetCubicSplineFit() const;

protected:
    std::vector<ATHit> _hits;
    size_t _startHitIndex = 0;
    size_t _endHitIndex = 0;
    float _approximateTrajectoryLength = 0.0f;
    float _averageCurvature = 0.0f;
    Eigen::Vector3f _centroidPoint;
    Eigen::Vector3f _mainDirection;
    ATCubicSplineFit _cubicSplineFit;
};

#endif
