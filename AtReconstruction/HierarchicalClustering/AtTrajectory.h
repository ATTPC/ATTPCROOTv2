#ifndef AtTRAJECTORY_HH
#define AtTRAJECTORY_HH

#include <vector>
#include <Eigen/Core>

#include "AtCubicSplineFit.h"
#include "AtHit.h"

class AtTrajectory {
public:
   AtTrajectory(std::vector<AtHit> const &hits, Eigen::Vector3f const &centroidPoint,
                Eigen::Vector3f const &mainDirection, AtCubicSplineFit const &cubicSplineFit);

   std::vector<AtHit> const &GetHits() const;

   static float GetPositionOnMainDirection(Eigen::Vector3f const &centroidPoint, Eigen::Vector3f const &mainDirection,
                                           Eigen::Vector3f const &point);
   float GetPositionOnMainDirection(Eigen::Vector3f const &point) const;

   Eigen::Vector3f const &GetCentroidPoint() const;
   Eigen::Vector3f const &GetMainDirection() const;
   AtCubicSplineFit const &GetCubicSplineFit() const;

protected:
   std::vector<AtHit> _hits;
   Eigen::Vector3f _centroidPoint;
   Eigen::Vector3f _mainDirection;
   AtCubicSplineFit _cubicSplineFit;
};

#endif
