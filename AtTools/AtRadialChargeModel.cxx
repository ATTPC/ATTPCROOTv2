#include "AtRadialChargeModel.h"

#include "AtDigiPar.h"

#include <FairLogger.h>

#include <Math/Point2D.h>
#include <Math/Vector2D.h>
#include <Math/Vector3D.h>

#include <cmath>

constexpr auto c = 29979.2;  //< c in cm/us
constexpr auto c2 = c * c;   //< c^2
constexpr auto me = 0.511e6; // mass of e- in eV

using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;
using XYPoint = ROOT::Math::XYPoint;
using XYVector = ROOT::Math::XYVector;

AtRadialChargeModel::AtRadialChargeModel(EFieldPtr eField) : AtSpaceChargeModel(), GetEField(eField) {}

XYZPoint AtRadialChargeModel::CorrectSpaceCharge(const XYZPoint &input)
{
   return SolveEqn(input / 10, true) * 10;
}

XYZPoint AtRadialChargeModel::ApplySpaceCharge(const XYZPoint &input)
{
   return SolveEqn(input / 10, false) * 10;
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

      auto Efield = GetEField(pos, z);
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

void AtRadialChargeModel::LoadParameters(AtDigiPar *par)
{
   if (par == nullptr)
      return;

   SetEField(par->GetEField());
   SetDriftVelocity(par->GetDriftVelocity());
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
