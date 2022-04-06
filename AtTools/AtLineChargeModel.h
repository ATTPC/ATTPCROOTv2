// Function to take in uncorrected position (VectorPoint3D) and return corrected position (vice/versa)
#ifndef ATLINECHARGEMODEL_H
#define ATLINECHARGEMODEL_H

#include <Math/Point3Dfwd.h>
#include <Rtypes.h>
#include <Math/Point3D.h>

#include "AtSpaceChargeModel.h"

class TBuffer;
class TClass;
class TMemberInspector;

using XYZPoint = ROOT::Math::XYZPoint;

class AtLineChargeModel : public AtSpaceChargeModel {

private:
   Double_t fLambda, fField;

public:
   // units are ...
   AtLineChargeModel(Double_t inputLambda = 5.28e-8, Double_t inputField = 70000);
   ~AtLineChargeModel() = default;

   virtual XYZPoint CorrectSpaceCharge(const XYZPoint &directInputPosition) override;
   virtual XYZPoint ApplySpaceCharge(const XYZPoint &reverseInputPosition) override;

   ClassDefOverride(AtLineChargeModel, 1);
};

#endif
