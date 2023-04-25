#ifndef ATRADIALCHARGEMODEL_H
#define ATRADIALCHARGEMODEL_H

#include "AtSpaceChargeModel.h"

#include <Rtypes.h> // for Double_t

#include <functional>
class AtDigiPar;

/**
 * @brief Space charge model from arbitrary radial E-field,
 *
 * Calculates the displacement of electrons from an arbirary radial E-field.
 * Uses AtDigiPar to get gas parameters of interest.
 *
 */
class AtRadialChargeModel : public AtSpaceChargeModel {
public:
   using EFieldPtr = std::function<double(double rho, double z)>;

private:
   /**
    * @brief Function to return the magnitude of the electric field.
    * rho and z are in units of cm
    * Returned field is V/cm
    */
   EFieldPtr GetEField{nullptr};

   Double_t fEFieldZ{700};             //< Magnitude of electric field in Z direction [V/cm]
   Double_t fDriftVel{0.815};          //< Drift velocity of electron in gas [cm/us]
   Double_t fMobilityElec{1.16429e-3}; //< Mobility of electron (calculated from drift velocity) [cm2/V/us]
   Double_t fStepSize{1e-4};           //< Step size for solving differential equation [us]
   XYZPoint fWindow{0, 0, 0};          //<Beam location at window in mm
   XYZPoint fPadPlane{0, 0, 1000};     //<Beam location at pad plane in mm

public:
   AtRadialChargeModel(EFieldPtr efield);

   virtual XYZPoint CorrectSpaceCharge(const XYZPoint &directInputPosition) override;
   virtual XYZPoint ApplySpaceCharge(const XYZPoint &reverseInputPosition) override;

   void SetDistortionField(EFieldPtr field) { GetEField = field; }
   void SetStepSize(double setSize) { fStepSize = setSize; }
   void SetEField(double field);
   void SetDriftVelocity(double v);
   void LoadParameters(const AtDigiPar *par) override;

private:
   XYZPoint SolveEqn(XYZPoint ele, bool correction);
};

class AtLineChargeZDep {

   double fLambda;

public:
   AtLineChargeZDep(double l) : fLambda(l) {}

   double operator()(double rho, double z)
   {
      constexpr double rBeam = 2.0 / 100;    // in *m*
      constexpr double eps = 8.85418782E-12; // SI
      constexpr double pi = 3.14159265358979;
      constexpr double eps2pi = 2 * pi * eps;
      rho /= 100.; // Convert units from cm to m
      z /= 100.;   // Convert units from cm to m

      double field;
      if (rho > rBeam)
         field = fLambda / eps2pi / rho * (z / 1.); // v/m
      else
         // field = lambda / eps2pi / rBeam / rBeam * rho * (z/1.); // v/m
         field = 0;
      return field / 100.; // V/cm
   }
};
#endif /* ATRADIALCHARGEMODEL_H */
