#ifndef ATELOSSBETHEBLOCH_H
#define ATELOSSBETHEBLOCH_H

#include "AtELossModel.h"
#include "AtSpline.h"

#include <cmath>
#include <vector>

namespace AtTools {

/**
 * Analytic energy loss model based on the Bethe-Bloch equation.
 * Supports heavy charged particles (protons, pions, heavier ions) using the
 * standard PDG formula (PDG 2022, Eq. 34.1), and electrons using the modified
 * formula with Møller exchange corrections (Leo 1994, Eq. 2.38).
 * Straggling uses the Bohr approximation.
 * Internal units: MeV/mm.
 */
class AtELossBetheBloch : public AtELossModel {
protected:
   static constexpr double kK = 0.307075;        // MeV cm²/mol (4πNₐrₑ²mₑc²)
   static constexpr double kM_e = 0.51099895069; // electron mass MeV/c²

   double fPart_q;    // projectile charge in units of e
   double fPart_mass; // projectile rest mass in MeV/c²
   int fMat_Z;        // target atomic number
   int fMat_A;        // target mass number (g/mol)
   double fI_MeV;     // mean excitation energy in MeV

   tk::spline fdXdE; // spline of dx/dE vs. energy; integral cached for O(log n) GetRange

public:
   /**
    * @param part_q    Charge of projectile in elementary charge units (e.g. 1 for proton).
    * @param part_mass Rest mass of projectile in MeV/c² (e.g. 938.272 for proton).
    * @param mat_Z     Atomic number of target material.
    * @param mat_A     Mass number of target material (g/mol).
    * @param density   Density of target material in g/cm³.
    * @param I_eV      Mean excitation energy in eV. If ≤ 0, uses Bloch approx: I ≈ 13.5·Z eV.
    */
   AtELossBetheBloch(double part_q, double part_mass, int mat_Z, int mat_A, double density, double I_eV = -1);

   void SetMaterial(int mat_Z, int mat_A, double density, double I_eV = -1);
   void SetI(double I_eV);
   virtual void SetDensity(double density) override;

   /**
    * (Re)build the internal dx/dE spline over [E_min, E_max] with nPoints log-spaced samples.
    * Called automatically by the constructor; call again after changing particle/material.
    */
   void BuildSpline(double E_min_MeV = 0.01, double E_max_MeV = 1000.0, int nPoints = 200);

   virtual double GetdEdx(double energy) const override;
   virtual double GetRange(double energyIni, double energyFin = 0) const override;
   virtual double GetEnergyLoss(double energyIni, double distance) const override;
   virtual double GetEnergy(double energyIni, double distance) const override;
   virtual double GetElossStraggling(double energyIni, double energyFin) const override;
   virtual double GetdEdxStraggling(double energyIni, double energyFin) const override;

private:
   bool IsElectron() const { return std::abs(fPart_mass - kM_e) < 0.01; }
   double GetdEdx_formula(double energy) const;  // analytic Bethe-Bloch, used by BuildSpline
   double GetdEdx_heavy(double energy) const;    // heavy particle variant (PDG 2022, Eq. 34.1)
   double GetdEdx_electron(double energy) const; // electron variant (Leo 1994, Eq. 2.38)
};

} // namespace AtTools

#endif // ATELOSSBETHEBLOCH_H
