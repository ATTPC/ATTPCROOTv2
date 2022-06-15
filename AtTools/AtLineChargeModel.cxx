
#include "AtLineChargeModel.h"

#include "AtDigiPar.h"

#include <FairLogger.h>

#include <Rtypes.h>

#include <cmath>
#include <memory>

using XYZPoint = ROOT::Math::XYZPoint;
using RZPPoint = ROOT::Math::RhoZPhiPoint;

AtLineChargeModel::AtLineChargeModel(Double_t inputLambda, Double_t inputField)
   : fLambda(inputLambda), fField(inputField)
{
}

XYZPoint AtLineChargeModel::CorrectSpaceCharge(const XYZPoint &directInputPosition)
{
   auto rhoI = directInputPosition.Rho() * 1e-3;
   auto delZ = directInputPosition.Z() * 1e-3;

   // 256787 is numerical value of constant factors in distortion expression
   auto rhoF = sqrt(rhoI * rhoI + 2 * (256787 / fField) * (70000) * fLambda * delZ) * 1e3;
   LOG(debug) << "Correcting rho: " << rhoI * 1e3 << " to " << rhoF;
   return (XYZPoint)RZPPoint(rhoF, directInputPosition.Z(), directInputPosition.Phi());
}

XYZPoint AtLineChargeModel::ApplySpaceCharge(const XYZPoint &reverseInputPosition)
{
   auto rhoI = reverseInputPosition.Rho() * 1e-3;
   auto delZ = reverseInputPosition.Z() * 1e-3;

   // 256787 is numerical value of constant factors in distortion expression
   auto rhoF = sqrt(rhoI * rhoI - 2 * (256787 / fField) * (70000) * fLambda * delZ) * 1e3;
   return (XYZPoint)RZPPoint(rhoF, reverseInputPosition.Z(), reverseInputPosition.Phi());
}

void AtLineChargeModel::LoadParameters(AtDigiPar *par)
{
   if (par == nullptr)
      return;
   auto field = par->GetEField(); //[V/cm]
   fField = field * 100.;         // v/m
}

ClassImp(AtLineChargeModel);
