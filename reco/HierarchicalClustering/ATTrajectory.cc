#include "ATTrajectory.hh"

ATTrajectory::ATTrajectory()
{
    // NOOP
}

ATTrajectory::ATTrajectory(std::vector<ATHit> hits, size_t startHitIndex, size_t endHitIndex, float approximateTrajectoryLength, float averageCurvature, Eigen::Vector3f centroidPoint, Eigen::Vector3f mainDirection)
    : ATTrajectory()
{
    this->_hits = hits;
    this->_startHitIndex = startHitIndex;
    this->_endHitIndex = endHitIndex;
    this->_approximateTrajectoryLength = approximateTrajectoryLength;
    this->_averageCurvature = averageCurvature;
    this->_centroidPoint = centroidPoint;
    this->_mainDirection = mainDirection;
}

std::vector<ATHit> const &ATTrajectory::GetHits() const
{
    return this->_hits;
}

size_t const &ATTrajectory::GetStartHitIndex() const
{
    return this->_startHitIndex;
}

ATHit const &ATTrajectory::GetStartHit() const
{
    return this->GetHits()[this->GetStartHitIndex()];
}

size_t const &ATTrajectory::GetEndHitIndex() const
{
    return this->_endHitIndex;
}

ATHit const &ATTrajectory::GetEndHit() const
{
    return this->GetHits()[this->GetEndHitIndex()];
}

float const &ATTrajectory::GetApproximateTrajectoryLength() const
{
    return this->_approximateTrajectoryLength;
}

float const &ATTrajectory::GetAverageCurvature() const
{
    return this->_averageCurvature;
}

Eigen::Vector3f const &ATTrajectory::GetCentroidPoint() const
{
    return this->_centroidPoint;
}

Eigen::Vector3f const &ATTrajectory::GetMainDirection() const
{
    return this->_mainDirection;
}
