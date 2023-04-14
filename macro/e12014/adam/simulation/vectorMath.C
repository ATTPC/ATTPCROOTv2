#ifndef __CLING__
#include <Math/Rotation3D.h>
#include <Math/Vector3D.h>
#include <Math/Vector4D.h>
#include <TMath.h>
#endif

using namespace ROOT::Math;

// Get rotation to vec is taken to the z-axis in the new coordinate system.
AxisAngle GetRotation(XYZVector vec)
{
   XYZVector z(0, 0, 1);
   double angle = TMath::ACos(z.Dot(vec.Unit()));

   auto rotVec = vec.Unit().Cross(z);
   return AxisAngle(rotVec, angle);
}

void vectorMath()
{
   XYZVector newZ(1, 0, 0);
   auto rot = GetRotation(newZ);
   std::cout << rot << std::endl;

   XYZVector vec(1, 1, 0);

   std::cout << vec << std::endl;
   std::cout << rot(vec) << std::endl;
}
