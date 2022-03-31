
#include "AtLineChargeModel.h"

// stdlib headers
#include <iostream>
#include "Math/Point3D.h"
#include <iostream>
#include "TObject.h"

using XYZPoint = ROOT::Math::XYZPoint;
using RZPPoint = ROOT::Math::RhoZPhiPoint;

AtLineChargeModel::AtLineChargeModel(Double_t inputLambda, Double_t inputField) : lambda(inputLambda), field(inputField)
{
}

XYZPoint AtLineChargeModel::DirectCorrection(const XYZPoint &directInputPosition)
{
   auto rhoI = directInputPosition.Rho() * 1e-3;
   auto delZ = directInputPosition.Z() * 1e-3;

   // 256787 is numerical value of constant factors in distortion expression

   auto rhoF = sqrt(rhoI * rhoI + (256787 / field) * (70000) * lambda * delZ) * 1e3;
   // RZPPoint convertPos(rhoF, directInputPosition.Z(), directInputPosition.Phi());
   // XYZPoint newPosition(convertPos.X(), convertPos.Y(), convertPos.Z());
   return (XYZPoint)RZPPoint(rhoF, directInputPosition.Z(), directInputPosition.Phi());
}

XYZPoint AtLineChargeModel::ReverseCorrection(const XYZPoint &reverseInputPosition)
{
   auto rhoI = reverseInputPosition.Rho();
   auto delZ = reverseInputPosition.Z();

   // 256787 is numerical value of constant factors in distortion expression

   auto rhoF = sqrt(rhoI * rhoI - (256787 / field) * (70000) * lambda * delZ) * 1e3;
   // RZPPoint convertPos(rhoF, reverseInputPosition.Z(), reverseInputPosition.Phi());
   // XYZPoint newPosition(convertPos.X(), convertPos.Y(), convertPos.Z());
   return (XYZPoint)RZPPoint(rhoF, reverseInputPosition.Z(), reverseInputPosition.Phi());
}

ClassImp(AtLineChargeModel);