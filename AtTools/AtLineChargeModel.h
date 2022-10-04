// Function to take in uncorrected position (VectorPoint3D) and return corrected position (vice/versa)
#ifndef ATLINECHARGEMODEL_H
#define ATLINECHARGEMODEL_H

#include "AtSpaceChargeModel.h"

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
class AtDigiPar;

using XYZPoint = ROOT::Math::XYZPoint;

class AtLineChargeModel : public AtSpaceChargeModel {

private:
   double fLambda; //< Magnitude of line charge [C/m]
   double fField;  //< Magnitude of drift field [V/m]

public:
   // units are ...
   AtLineChargeModel(double inputLambda = 5.28e-8, double inputfield = 70000);
   ~AtLineChargeModel() = default;

   void SetDriftField(double field) { fField = field; }
   virtual XYZPoint CorrectSpaceCharge(const XYZPoint &directInputPosition) override;
   virtual XYZPoint ApplySpaceCharge(const XYZPoint &reverseInputPosition) override;
   virtual void LoadParameters(AtDigiPar *par) override;
};

#endif
