#include "AtPatternRay.h"

#include "AtPattern.h" // for AtPattern, AtPatterns

#include <Math/Point3D.h>  // for operator+, operator-
#include <Math/Vector3D.h> // for DisplacementVector3D, operator*
#include <TEveLine.h>

#include <cmath> // for cos, sin, pow, sqrt, acos, atan, fabs

class TEveElement;

using namespace AtPatterns;

ClassImp(AtPatternRay);

AtPatternRay::AtPatternRay() : AtPatternLine() {}

TEveElement *AtPatternRay::GetEveElement() const
{
   return AtPattern::GetEveLine(0, 1000, 100);
}

AtPatternRay::XYZPoint AtPatternRay::ClosestPointOnPattern(const XYZPoint &point) const
{
   auto t = parameterAtPoint(point);
   return GetPointAt(t);
}

Double_t AtPatternRay::DistanceToPattern(const XYZPoint &point) const
{
   auto vec = ClosestPointOnPattern(point) - point;
   return vec.R();
}

AtPatternRay::XYZPoint AtPatternRay::GetPointAt(double z) const
{
   if (z > 0)
      return GetPoint() + z * GetDirection();
   else
      return GetPoint();
}
void AtPatternRay::DefinePattern(XYZPoint fPoint, XYZVector fDirection)
{
   if (fDirection.Z() != 0)
      fDirection /= fabs(fDirection.Z());
   fPatternPar = {fPoint.X(), fPoint.Y(), fPoint.Z(), fDirection.X(), fDirection.Y(), fDirection.Z()};
}
