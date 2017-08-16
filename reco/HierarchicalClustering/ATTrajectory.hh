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
        Eigen::Vector3f const &centroidPoint,
        Eigen::Vector3f const &mainDirection,
        ATCubicSplineFit const &cubicSplineFit
    );

    std::vector<ATHit> const &GetHits() const;

    static float GetPositionOnMainDirection(Eigen::Vector3f const &centroidPoint, Eigen::Vector3f const &mainDirection, Eigen::Vector3f const &point);
    float GetPositionOnMainDirection(Eigen::Vector3f const &point) const;

    Eigen::Vector3f const &GetCentroidPoint() const;
    Eigen::Vector3f const &GetMainDirection() const;
    ATCubicSplineFit const &GetCubicSplineFit() const;

protected:
    std::vector<ATHit> _hits;
    Eigen::Vector3f _centroidPoint;
    Eigen::Vector3f _mainDirection;
    ATCubicSplineFit _cubicSplineFit;
};

#endif
