#include "ATTrajectory.hh"

ATTrajectory::ATTrajectory(
    std::vector<ATHit> const &hits,
    Eigen::Vector3f const &centroidPoint,
    Eigen::Vector3f const &mainDirection,
    ATCubicSplineFit const &cubicSplineFit)
    : _cubicSplineFit(cubicSplineFit)
{
    this->_hits = hits;
    this->_centroidPoint = centroidPoint;
    this->_mainDirection = mainDirection;
}

std::vector<ATHit> const &ATTrajectory::GetHits() const
{
    return this->_hits;
}

float ATTrajectory::GetPositionOnMainDirection(Eigen::Vector3f const &centroidPoint, Eigen::Vector3f const &mainDirection, Eigen::Vector3f const &point)
{
    return mainDirection.dot(point - centroidPoint);
}

float ATTrajectory::GetPositionOnMainDirection(Eigen::Vector3f const &point) const
{
    return ATTrajectory::GetPositionOnMainDirection(this->GetCentroidPoint(), this->GetMainDirection(), point);
}

Eigen::Vector3f const &ATTrajectory::GetCentroidPoint() const
{
    return this->_centroidPoint;
}

Eigen::Vector3f const &ATTrajectory::GetMainDirection() const
{
    return this->_mainDirection;
}

ATCubicSplineFit const &ATTrajectory::GetCubicSplineFit() const
{
    return this->_cubicSplineFit;
}
