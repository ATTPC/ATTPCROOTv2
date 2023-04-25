// Function to take in uncorrected position (VectorPoint3D) and return corrected position (vice/versa)
#ifndef ATSPACECHARGEMODEL_H
#define ATSPACECHARGEMODEL_H

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
class AtDigiPar;

class AtSpaceChargeModel {
protected:
   using XYZPoint = ROOT::Math::XYZPoint;

   XYZPoint fWindow{0, 0, 0};      //<Beam location at window in mm
   XYZPoint fPadPlane{0, 0, 1000}; //<Beam location at pad plane in mm

public:
   virtual ~AtSpaceChargeModel() = default;
   /**
    * @brief Using model correct for space charge.
    *
    * Used correct for space charge in data.
    * @param[in] pos Position charge hits the pad plane [mm].
    * @return Position charge was deposited in the detector [mm].
    */
   virtual XYZPoint CorrectSpaceCharge(const XYZPoint &position) = 0;

   /**
    * @brief Using model add space charge effect.
    *
    * Used to simulate the effect of this space charge model.
    * @param[in] pos Position charge was deposited in detector [mm].
    * @return Position charge hits the pad plane [mm].
    */
   virtual XYZPoint ApplySpaceCharge(const XYZPoint &position) = 0;

   /**
    * @brief Load common parameters from AtDigiPar.
    *
    * Will load any parameters used by the model from the parameter file attached
    * to the run.
    */
   virtual void LoadParameters(const AtDigiPar *par) = 0;

   void SetBeamLocation(XYZPoint window, XYZPoint padPlane)
   {
      fWindow = window;
      fPadPlane = padPlane;
   }

protected:
   XYZPoint OffsetForBeam(XYZPoint point);
   XYZPoint UndoOffsetForBeam(XYZPoint point);
};

#endif //#ifndef ATSPACECHARGEMODEL_H
