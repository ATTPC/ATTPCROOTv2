#ifndef ATELOSSCATIMA_H
#define ATELOSSCATIMA_H
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtELossModel.h"

#include <catima/catima.h>
#include <catima/config.h>
#include <memory>
#include <tuple>
#include <vector>

namespace AtTools {

class AtELossCATIMA : public AtELossModel {
protected:
   std::unique_ptr<catima::Material> fMaterial{nullptr};
   std::unique_ptr<catima::Projectile> fProjectile{nullptr};

   double fProjectileMassAmu{-1}; /// Mass of the projectile in amu (atomic mass units).

   double fRangeStepSize{0.1}; // mm
   catima::Config fConfig{catima::default_config};

public:
   /**
    * Initializer of the CATIMA AtELossModel wrapper.
    * @param[in] density Density of the material (g/cm^2).
    * @param[in] materialComponents Components of the material. They are passed as a vector of tuples (A, Z,
    * stoichiometry).
    */
   AtELossCATIMA(double density) : AtELossModel(density) {}
   AtELossCATIMA(double density, std::vector<std::tuple<int, int, int>> materialComponents);
   AtELossCATIMA(double density, const catima::Material &material)
      : AtELossModel(density), fMaterial(std::make_unique<catima::Material>(material))
   {
   }

   virtual double GetdEdx(double energy) const override;
   virtual double GetRange(double energyIni, double energyFin = 0) const override;
   virtual double GetEnergyLoss(double energyIni, double distance) const override
   {
      return energyIni - GetEnergy(energyIni, distance);
   }
   virtual double GetEnergy(double energyIni, double distance) const override;

   virtual double GetRangeVariance(double energy) const override;
   virtual double GetElossStraggling(double energyIni, double energyFin) const override;
   virtual double GetdEdxStraggling(double energyIni, double energyFin) const override;

   virtual std::vector<std::pair<double, double>>
   GetBraggCurve(double energy, double rangeStepSize = 0, double totalFractionELoss = 0.001) const override;

   /**
    * Setter of the catima projectile used for calculations.
    * @param[in] A Mass number of the projectile.
    * @param[in] Z Charge number of the projectile.
    * @param[in] massAmu Mass of the projectile in amu.
    */
   void SetProjectile(double A, double Z, double massAmu)
   {
      fProjectile = std::make_unique<catima::Projectile>(A, Z);
      fProjectileMassAmu = massAmu;
   }
   void SetMaterial(const catima::Material &material) { fMaterial = std::make_unique<catima::Material>(material); }
   void SetMaterial(std::vector<std::tuple<int, int, int>> materialComponents);

   /**
    * Setter of the range step size used for calculations. By default it is set to 0.1mm.
    * @param[in] stepSize The step size used for ranges. It must be input in mm.
    */
   void SetRangeStepSize(double stepSize) { fRangeStepSize = stepSize; }
   void SetConfig(catima::Config cfg) { fConfig = cfg; }
};

} // namespace AtTools

#endif
