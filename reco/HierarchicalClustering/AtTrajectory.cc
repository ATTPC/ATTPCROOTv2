#include "AtTrajectory.h"

AtTrajectory::AtTrajectory(
    std::vector<AtHit> const &hits,
    Eigen::Vector3f const &centroidPoint,
    Eigen::Vector3f const &mainDirection,
    AtCubicSplineFit const &cubicSplineFit)
    : _cubicSplineFit(cubicSplineFit)
{
    this->_hits = hits;
    this->_centroidPoint = centroidPoint;
    this->_mainDirection = mainDirection;
}

std::vector<AtHit> const &AtTrajectory::GetHits() const
{
    return this->_hits;
}

float AtTrajectory::GetPositionOnMainDirection(Eigen::Vector3f const &centroidPoint, Eigen::Vector3f const &mainDirection, Eigen::Vector3f const &point)
{
    return mainDirection.dot(point - centroidPoint);
}

float AtTrajectory::GetPositionOnMainDirection(Eigen::Vector3f const &point) const
{
    return AtTrajectory::GetPositionOnMainDirection(this->GetCentroidPoint(), this->GetMainDirection(), point);
}

Eigen::Vector3f const &AtTrajectory::GetCentroidPoint() const
{
    return this->_centroidPoint;
}

Eigen::Vector3f const &AtTrajectory::GetMainDirection() const
{
    return this->_mainDirection;
}

AtCubicSplineFit const &AtTrajectory::GetCubicSplineFit() const
{
    return this->_cubicSplineFit;
}
