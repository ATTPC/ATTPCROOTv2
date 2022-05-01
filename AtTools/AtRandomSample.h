#ifndef ATRANDOMSAMPLE_H
#define ATRANDOMSAMPLE_H
#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint

#include <iosfwd> // for ostream
#include <vector>
class AtHit;

/**
 * Class to randomly sample AtHits, or vectors of arbitrary types
 */
class AtRandomSample {
public:
   enum class SampleMethod { kUniform = 0, kGaussian = 1, kWeighted = 2, kWeightedGaussian = 3 };
   using XYZPoint = ROOT::Math::XYZPoint;

   static std::vector<XYZPoint> SamplePoints(int N, const std::vector<AtHit> &hits, SampleMethod mode);

private:
   class AtRandomSample_impl;
};

std::ostream &operator<<(std::ostream &os, const AtRandomSample::SampleMethod &t);

#endif //#ifndef ATRANDOMSAMPLE_H
