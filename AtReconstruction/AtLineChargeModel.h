
// Function to take in uncorrected position (VectorPoint3D) and return corrected position (vice/versa)
#ifndef ATLINECHARGEMODEL_H
#define ATLINECHARGEMODEL_H

// Conversion Help Class
#include "AtSpaceChargeModel.h"
#include "Math/Point3D.h"
#include "TObject.h"

using XYZPoint = ROOT::Math::XYZPoint;

class AtLineChargeModel : public AtSpaceChargeModel {

private:
   Double_t lambda, field;

public:
   // units are ...
   AtLineChargeModel(Double_t inputLambda = 5.28e-8, Double_t inputField = 70000);
   ~AtLineChargeModel() = default;

   virtual XYZPoint DirectCorrection(const XYZPoint &directInputPosition) override;
   virtual XYZPoint ReverseCorrection(const XYZPoint &reverseInputPosition) override;

   ClassDefOverride(AtLineChargeModel, 1);
};

#endif