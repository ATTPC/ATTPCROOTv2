#ifndef ATELOSSMODEL_H
#define ATELOSSMODEL_H

namespace AtTools {

/**
 * Class representing the energy loss of a particle through some material.
 * Derived classes can represent models from different sources (SRIM, etc).
 * Internal units are Mev/mm
 *
 * Based on a combination of Nabin Rijal's AtELossManager and the EnergyLoss class
 * (https://github.com/joshhooker/EnergyLossClass) which is released unter the MIT Licsense (copyright Joshua Hooker).
 */

class AtELossModel {
protected:
   /**
    *  Density of the target in mg/cm^3 if known (used to scale E-loss over different ranges).
    *  This is the density used in the internal model (set on construction).
    */
   double fDensityIni;
   /**
    *  Density of the target in mg/cm^3 we are calcualting energy losses for. Energy loss is scaled
    *  using this value and fDensityIni.
    */
   double fDensity;

   double fdEdxScale{1};

public:
   AtELossModel(double density) : fDensityIni(density), fDensity(fDensityIni){};
   virtual ~AtELossModel() = default;

   void SetDensity(double density);
   /**
    * Get the stopping power in MeV/mm
    */
   virtual double GetdEdx(double energy) const = 0;

   /**
    * Get the range of the particle in the material.
    */
   virtual double GetRange(double energyIni, double energyFin = 0) const = 0;

   /**
    * Get the energy loss over some distance (in mm).
    */
   virtual double GetEnergyLoss(double energyIni, double distance) const = 0;

   /**
    * Get the energy of particle after traveling some distance (in mm).
    * If the distance is negative, then returns the energy the particle had to
    * reach energyIni after distance.
    */
   virtual double GetEnergy(double energyIni, double distance) const = 0;

protected:
   void SetIniDensity(double density)
   {
      fDensityIni = density;
      fDensity = density;
   }
};
} // namespace AtTools

#endif
