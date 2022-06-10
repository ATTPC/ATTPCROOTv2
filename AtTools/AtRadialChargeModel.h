#ifndef ATRADIALCHARGEMODEL_H
#define ATRADIALCHARGEMODEL_H

#include "AtSpaceChargeModel.h"

#include <Math/Point2Dfwd.h>
#include <Math/Vector2Dfwd.h>
/**
 * @brief Space charge model from arbitrary radial E-field,
 *
 * Calculates the displacement of electrons from an arbirary radial E-field.
 * Uses AtDigiPar to get gas parameters of interest.
 *
 */
class AtRadialChargeModel : public AtSpaceChargeModel {
public:
   using EFieldPtr = std::add_pointer_t<double(double rho, double z)>;

private:
   /**
    * @brief Function to return the magnitude of the electric field.
    * rho and z are in units of cm
    * Returned field is V/cm
    */
   EFieldPtr GetEField{nullptr};

   Double_t fEFieldZ{7000};            //< Magnitude of electric field in Z direction [V/cm]
   Double_t fDriftVel{0.82};           //< Drift velocity of electron in gas [cm/us]
   Double_t fMobilityElec{1.17143e-3}; //< Mobility of electron (calculated from drift velocity) [cm2/V/us]
   Double_t fStepSize{1e-4};           //< Step size for solving differential equation [us]

public:
   AtRadialChargeModel(EFieldPtr efield);

   virtual XYZPoint CorrectSpaceCharge(const XYZPoint &directInputPosition) override;
   virtual XYZPoint ApplySpaceCharge(const XYZPoint &reverseInputPosition) override;

   void SetStepSize(double setSize) { fStepSize = setSize; }
   void SetEField(double field);
   void SetDriftVelocity(double v);
   void LoadParameters(AtDigiPar *par) override;

private:
   XYZPoint SolveEqn(XYZPoint ele, bool correction);
};
#endif /* ATRADIALCHARGEMODEL_H */
