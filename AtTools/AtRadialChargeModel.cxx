#include "AtRadialChargeModel.h"

#include "AtDigiPar.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Math/Point2Dfwd.h> // for XYPoint
#include <Math/Point3D.h>    // for PositionVector3D
#include <Math/Point3Dfwd.h> // for RhoZPhiPoint, XYZPoint
#include <Math/Vector2D.h>
#include <Math/Vector2Dfwd.h> // for XYVector
#include <Math/Vector3D.h>
#include <Math/Vector3Dfwd.h> // for XYZVector

#include <cmath>
#include <memory>  // for allocator
#include <utility> // for move

constexpr auto c = 29979.2;  //< c in cm/us
constexpr auto c2 = c * c;   //< c^2
constexpr auto me = 0.511e6; // mass of e- in eV

using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;
using XYPoint = ROOT::Math::XYPoint;
using XYVector = ROOT::Math::XYVector;

AtRadialChargeModel::AtRadialChargeModel(EFieldPtr eField) : AtSpaceChargeModel(), GetEField(std::move(eField)) {}

XYZPoint AtRadialChargeModel::CorrectSpaceCharge(const XYZPoint &input)
{
   auto offsetHit = OffsetForBeam(input);
   LOG(debug) << input << " to " << offsetHit;
   auto corrHit = SolveEqn(offsetHit / 10, true) * 10;
   return UndoOffsetForBeam(corrHit);
}

XYZPoint AtRadialChargeModel::ApplySpaceCharge(const XYZPoint &input)
{
   auto offsetHit = OffsetForBeam(input);
   auto corrHit = SolveEqn(offsetHit / 10, false) * 10;
   return UndoOffsetForBeam(corrHit);
}

// Assumes units are cm
XYZPoint AtRadialChargeModel::SolveEqn(XYZPoint ele, bool correct)
{

   // Verify step size is logical
   auto minStepSize = 2 * fMobilityElec * me / c2;
   if (fStepSize < minStepSize) {
      LOG(error) << "Using unphysical step size: " << fStepSize << " reseting to minimum step size:" << minStepSize;
      fStepSize = minStepSize;
   }

   // Drift to pad plane in z/vd
   double timeToDrift = ele.Z() / fDriftVel; // us
   int nBins = std::floor(timeToDrift / fStepSize);
   LOG(debug) << "Drifting to " << ele.Z() << " in " << nBins << " steps of size " << fStepSize;

   double pos = ele.rho();

   // Calculate transporting from point to the pad plane
   for (int i = 0; i < nBins; ++i) {

      // Z = 0 is pad plane
      auto z = ele.Z() - i * fStepSize * fDriftVel;
      if (z < 0) {
         LOG(error) << "Space charge correction tried to bypass the pad plane!";
         break;
      }

      double Efield = 0;
      if (GetEField == nullptr) {
         LOG(fatal) << "The distrotion field was never set!";
      } else
         Efield = GetEField(pos, z);

      LOG(debug2) << "Field " << Efield << " V/cm rho: " << pos << " cm and z: " << z << " cm.";
      if (!correct)
         Efield *= -1;

      // Method assuming we are always at drift velocity
      // This is true based on our check of the time step
      double v = Efield * fMobilityElec;

      // Update the position of the electron
      pos += fStepSize * v;
      if (pos < 0) {
         pos = 0;
      }
   }

   // Perform the final step using the remaining time
   auto dT = timeToDrift - nBins * fStepSize;
   auto z = dT * fDriftVel;
   auto Efield = GetEField(pos, z);
   if (!correct)
      Efield *= -1;
   double v = Efield * fMobilityElec;
   pos += dT * v;
   if (pos < 0) {
      pos = 0;
   }

   return XYZPoint(ROOT::Math::RhoZPhiPoint(pos, ele.Z(), ele.phi()));
}

void AtRadialChargeModel::LoadParameters(const AtDigiPar *par)
{
   if (par == nullptr)
      return;

   SetEField(par->GetEField() / 100.); // EField units in param are V/m. Need V/cm.
   SetDriftVelocity(par->GetDriftVelocity());
   LOG(debug) << "Setting mobility to: " << fMobilityElec;
}

void AtRadialChargeModel::SetEField(double field)
{
   fEFieldZ = field;
   fMobilityElec = fDriftVel / fEFieldZ;
}
void AtRadialChargeModel::SetDriftVelocity(double v)
{
   fDriftVel = v;
   fMobilityElec = fDriftVel / fEFieldZ;
}

/*
// Calculate the acceleration and update the velocity
auto a = Efield * c2 / me - c2 / fMobilityElec / me * v;
auto dV = fStepSize * a;
// v += fStepSize * dV;

// Method used by Juan (equivalent mathematically, not tested)
auto G = c2 / fMobilityElec;
v = (v * (1.0 - 0.5 * fStepSize * G / me) + fStepSize * (Efield * c2 / me)) / (1.0 + 0.5 * fStepSize * G / me);
*/
