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
   double fLambda{1.54e-8};   //< Magnitude of line charge [C/m]
   double fField{70000};      //< Magnitude of drift field [V/m]
   double fDetectorLength{1}; //< Length of detector [m]
   double fBeamRadius{.02};   //< Radius of beam region (don't correct inside this radius) [m]
   bool fLinearField{true};   //<If true the field grows linearly from the pad plane to the window.

public:
   AtLineChargeModel() = default;
   ~AtLineChargeModel() = default;

   virtual XYZPoint CorrectSpaceCharge(const XYZPoint &directInputPosition) override;
   virtual XYZPoint ApplySpaceCharge(const XYZPoint &reverseInputPosition) override;
   virtual void LoadParameters(const AtDigiPar *par) override;

   void SetDriftField(double field) { fField = field; }
   void SetConstantCharge() { fLinearField = false; }
   void SetLambda(double lambda) { fLambda = lambda; }
   void SetBeamRadius(double radius) { fBeamRadius = radius; }

   double GetLambda() const { return fLambda; }
   double GetField() const { return fField; }
   double GetDetectorLength() const { return fDetectorLength; }

public:
   double getDist2(double dZ);
};

#endif
