#ifndef ATELOSSMODEL_H
#define ATELOSSMODEL_H

#include <cmath>
#include <utility>
#include <vector>

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

   /**
    * @brief Get the range straggling for a given energy.
    *
    * @param energy The kinetic energy of the particle in MeV.
    * @return The range straggling (sigma) in mm.
    */
   virtual double GetRangeStraggling(double energy) const { return std::sqrt(GetRangeVariance(energy)); }

   /**
    * @brief Get the range variance for a given energy.
    * @param energy The kinetic energy of the particle in MeV.
    * @return The range variance  in mm^2.
    */
   virtual double GetRangeVariance(double energy) const { return 0; };

   /**
    * @brief Get the energy loss straggling for a particle between two energies.
    * @param energyIni The initial kinetic energy of the particle in MeV.
    * @param energyFin The final kinetic energy of the particle in MeV.
    *
    * @return The energy loss straggling in MeV.
    */
   virtual double GetEnergyLossStraggling(double energyIni, double energyFin) const { return 0; };

   /**
    * Get the Bragg curve for a given energy as a vector of (dE/dx, distance) pairs.
    * @param[in] energy The kinetic energy of the particle for which the curve is being computed for.
    * @param[in] rangeStepSize The step size for the distances the Bragg curve will be computed for in mm. Default value
    * is 0.1mm.
    * @param[in] totalFractionELoss Consider particle stopped when energy drops below energy*totalFractionELoss.
    * @return A vector of pairs (dE/dx, distance) representing the Bragg curve (MeV/mm, mm).
    */
   virtual std::vector<std::pair<double, double>>
   GetBraggCurve(double energy, double rangeStepSize = 0.1, double totalFractionELoss = 0.001) const;

protected:
   void SetIniDensity(double density)
   {
      fDensityIni = density;
      fDensity = density;
   }
};
} // namespace AtTools

#endif
