
#include "AtLineChargeModel.h"

#include <math.h>
#include <memory>

#include <Rtypes.h>
#include "fairlogger/Logger.h"

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
   auto rhoF = sqrt(rhoI * rhoI + (256787 / fField) * (70000) * fLambda * delZ) * 1e3;
   LOG(debug) << "Correcting rho: " << rhoI * 1e3 << " to " << rhoF;
   return (XYZPoint)RZPPoint(rhoF, directInputPosition.Z(), directInputPosition.Phi());
}

XYZPoint AtLineChargeModel::ApplySpaceCharge(const XYZPoint &reverseInputPosition)
{
   auto rhoI = reverseInputPosition.Rho();
   auto delZ = reverseInputPosition.Z();

   // 256787 is numerical value of constant factors in distortion expression
   auto rhoF = sqrt(rhoI * rhoI - (256787 / fField) * (70000) * fLambda * delZ) * 1e3;
   return (XYZPoint)RZPPoint(rhoF, reverseInputPosition.Z(), reverseInputPosition.Phi());
}

ClassImp(AtLineChargeModel);
