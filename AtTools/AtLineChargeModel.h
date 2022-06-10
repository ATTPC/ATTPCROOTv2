// Function to take in uncorrected position (VectorPoint3D) and return corrected position (vice/versa)
#ifndef ATLINECHARGEMODEL_H
#define ATLINECHARGEMODEL_H

#include "AtSpaceChargeModel.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
#include <Rtypes.h>

class TBuffer;
class TClass;
class TMemberInspector;
class AtDigiPar;

using XYZPoint = ROOT::Math::XYZPoint;

class AtLineChargeModel : public AtSpaceChargeModel {

private:
   Double_t fLambda; //< Magnitude of line charge [C/m]
   Double_t fField;  //< Magnitude of drift field [V/m]

public:
   // units are ...
   AtLineChargeModel(Double_t inputLambda = 5.28e-8, Double_t inputfield = 70000);
   ~AtLineChargeModel() = default;

   void SetDriftField(double field) { fField = field; }
   virtual XYZPoint CorrectSpaceCharge(const XYZPoint &directInputPosition) override;
   virtual XYZPoint ApplySpaceCharge(const XYZPoint &reverseInputPosition) override;
   virtual void LoadParameters(AtDigiPar *par) override;

   ClassDefOverride(AtLineChargeModel, 1);
};

#endif
