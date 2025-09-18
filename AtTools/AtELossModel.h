#ifndef ATELOSSMODEL_H
#define ATELOSSMODEL_H

#include <cmath>
#include <string>
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
    *  Density of the target in g/cm^3 we are calcualting energy losses for. Energy loss is scaled
    *  using this value and fDensityIni.
    */
   double fDensity;

   /**
    *  String containing the name of the specific ELossModel. Could be useful to keep track of which ELossModel was used
    *  in each case.
    */
   std::string fELossModelName;

   /**
    *  Particle PDG code for which this ELoss model works.
    */
   std::string fPDGCode{"none"};

   /**
    *  Charge in units of elemental charge that the particle has.
    */
   int fZ{0};

   /**
    *  Atomic mass number that the particle has.
    */
   int fA{0};

   /**
    *  Mass in amus that the particle has.
    */
   double fMassAmu{0};

public:
   AtELossModel(double density, std::string name = "noName") : fDensity(density), fELossModelName(name) {};
   virtual ~AtELossModel() = default;

   virtual void SetDensity(double density);
   virtual void SetELossModelName(std::string name);
   virtual void SetPDGCode(std::string pdg);
   virtual void SetChargeNumber(int z);
   virtual void SetAtomicMassNumber(int a);
   virtual void SetMassAmu(double mass);


   virtual double GetDensity();
   virtual std::string GetELossModelName();
   virtual std::string GetPDGCode();
   virtual int GetChargeNumber();
   virtual int GetAtomicMassNumber();
   virtual double GetMassAmu();

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
    * @brief Get the energy loss straggling for a particle over some distance.
    * @param energyIni The initial kinetic energy of the particle in MeV.
    * @param distance The distance traveled by the particle in mm.
    * @return The energy loss  variance in MeV^2.
    */
   virtual double GetElossStragglingDistance(double energyIni, double distance) const
   {
      double energyFin = GetEnergy(energyIni, distance);
      return GetElossStraggling(energyIni, energyFin);
   }

   /**
    * @brief Get the energy loss straggling for a particle between two energies.
    * @param energyIni The initial kinetic energy of the particle in MeV.
    * @param energyFin The final kinetic energy of the particle in MeV.
    *
    * @return The energy loss straggling in MeV.
    */
   virtual double GetElossStraggling(double energyIni, double energyFin) const = 0;

   /**
    * @brief Get the straggling in dE/dx for a particle between two energies.
    * @param energyIni The initial kinetic energy of the particle in MeV.
    * @param energyFin The final kinetic energy of the particle in MeV.
    *
    * Assumes the the dE/dx is constant over the range from energyIni to energyFin.
    * @return The straggling in dE/dx in MeV/mm.
    */
   virtual double GetdEdxStraggling(double energyIni, double energyFin) const = 0;
   /**
    * Get the Bragg curve for a given energy as a vector of (dE/dx, distance) pairs.
    * @param[in] energy The kinetic energy of the particle for which the curve is being computed for.
    * @param[in] rangeStepSize The step size for the distances the Bragg curve will be computed for in mm. Default value
    * is 0.1mm.
    * @param[in] totalFractionELoss Consider particle stopped when energy drops below energy*totalFractionELoss.
    * @param[in] minRange If minRange is 0, the Bragg curve is computed only until the particle has stopped. If it's
    * different than 0, then it will add dE/dx=0 until x=minRange after the actual Bragg curve in case it has not reached
    * that value yet. Default value is 0.
    * @return A vector of pairs (dE/dx, distance) representing the Bragg curve (MeV/mm, mm).
    */
   virtual std::vector<std::pair<double, double>>
   GetBraggCurve(double energy, double rangeStepSize = 0.1, double totalFractionELoss = 0.001, double minRange = 0) const;
};
} // namespace AtTools

#endif
