
#include "AtLineChargeModel.h"

#include "AtDigiPar.h"

#include <Math/Point3D.h>
#include <Rtypes.h>

#include <cmath>

using XYZPoint = ROOT::Math::XYZPoint;
using RZPPoint = ROOT::Math::RhoZPhiPoint;
namespace {
constexpr double eps = 8.85418782E-12; // SI
constexpr double pi = 3.14159265358979;
} // namespace

/// Units are m
double AtLineChargeModel::getDist2(double dZ)
{
   double prefactor = fLambda / pi / eps / fField;
   double dist2 = prefactor * dZ;
   if (fLinearField)
      dist2 *= dZ / 2 / fDetectorLength;
   return dist2;
}

XYZPoint AtLineChargeModel::CorrectSpaceCharge(const XYZPoint &directInputPosition)
{
   auto input = OffsetForBeam(directInputPosition);
   if (input.Rho() < fBeamRadius * 1000)
      return directInputPosition;

   auto delZ = input.Z();
   double dist2 = getDist2(delZ);
   double newRho = sqrt(input.Rho() * input.Rho() + dist2);

   XYZPoint output(RZPPoint(newRho, input.Z(), input.Phi()));
   return UndoOffsetForBeam(output);
}

XYZPoint AtLineChargeModel::ApplySpaceCharge(const XYZPoint &reverseInputPosition)
{
   auto input = OffsetForBeam(reverseInputPosition);
   if (input.Rho() < fBeamRadius * 1000) {
      return reverseInputPosition;
   }

   auto delZ = input.Z();         // in mm
   double dist2 = getDist2(delZ); // in mm;
   double newRho = sqrt(input.Rho() * input.Rho() - dist2);
   if (newRho < 0) {
      // std::cout << "newRho is " << newRho << std::endl;
      newRho = 0;
   }

   XYZPoint output(RZPPoint(newRho, input.Z(), input.Phi()));
   return UndoOffsetForBeam(output);
}

void AtLineChargeModel::LoadParameters(const AtDigiPar *par)
{
   if (par == nullptr)
      return;

   fField = par->GetEField(); // v/m
   fDetectorLength = par->GetZPadPlane() / 1000;
}

ClassImp(AtLineChargeModel);
