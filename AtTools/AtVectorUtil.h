#ifndef ATVECTORUTIL_H
#define ATVECTORUTIL_H

#include <Math/AxisAngle.h>
#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Math/Rotation3D.h>
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Math/VectorUtil.h>

#include <cassert>
namespace AtTools {

/**
 * Get the rotation matrix that will take vec and make it the z-axis. That is
 * `vec*GetRotationToZ(vec) == |vec|\hat{z}`.
 */
template <class Vector>
ROOT::Math::AxisAngle GetRotationToZ(const Vector &vec)
{
   using namespace ROOT::Math;
   XYZVector z(0, 0, 1);
   return AxisAngle(vec.Unit().Cross(z), VectorUtil::Angle(vec, z));
}

/**
 * Get the intersection (or point of closest approach) between two lines. The lines are parameterized
 * as point[i] + t*direction[i].
 */
template <class Vector1, class Vector2>
ROOT::Math::XYZPoint GetIntersection(const std::vector<Vector1> &point, const std::vector<Vector2> &direction)
{
   assert(point.size() > 1 && direction.size() > 1);
   auto n = direction[0].Cross(direction[1]);
   auto n0 = direction[0].Cross(n);
   auto n1 = direction[1].Cross(n);

   auto c0 = point[0] + (point[1] - point[0]).Dot(n1) / direction[0].Dot(n1) * direction[0];
   auto c1 = point[1] + (point[0] - point[1]).Dot(n0) / direction[1].Dot(n0) * direction[1];

   return ROOT::Math::XYZPoint((c0 + c1) / 2.);
}

} // namespace AtTools
#endif //#ifndef ATVECTORUTIL_H
