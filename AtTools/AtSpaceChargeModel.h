// Function to take in uncorrected position (VectorPoint3D) and return corrected position (vice/versa)
#ifndef ATSPACECHARGEMODEL_H
#define ATSPACECHARGEMODEL_H

#include "Math/Point3D.h"
#include "TObject.h"

using XYZPoint = ROOT::Math::XYZPoint;

class AtSpaceChargeModel : public TObject {

public:
   AtSpaceChargeModel();
   ~AtSpaceChargeModel() = default;

   virtual XYZPoint CorrectSpaceCharge(const XYZPoint &directInputPosition) = 0;
   virtual XYZPoint ApplySpaceCharge(const XYZPoint &reverseInputPosition) = 0;

   ClassDef(AtSpaceChargeModel, 1);
};

#endif //#ifndef ATSPACECHARGEMODEL_H
