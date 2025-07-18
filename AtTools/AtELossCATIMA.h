#ifndef ATELOSSCATIMA_H
#define ATELOSSCATIMA_H
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtELossModel.h"

#include <catima/catima.h>
#include <memory>
#include <tuple>
#include <vector>

namespace AtTools {

class AtELossCATIMA : public AtELossModel {
protected:
   std::unique_ptr<catima::Material> fMaterial{nullptr};
   std::unique_ptr<catima::Projectile> fProjectile{nullptr};

   double fProjectileMassUma{-1};

   double fRangeStepSize{0.1}; // mm

public:
   /**
    * Initializer of the CATIMA AtELossModel wrapper.
    * @param[in] density Density of the material.
    * @param[in] materialComponents Components of the material. They are passed as a vector of tuples (A, Z,
    * stoichiometry).
    */
   AtELossCATIMA(double density, std::vector<std::tuple<int, int, int>> materialComponents);

   virtual double GetdEdx(double energy) const override;
   virtual double GetRange(double energyIni, double energyFin = 0) const override;
   virtual double GetEnergyLoss(double energyIni, double distance) const override
   {
      return energyIni - GetEnergy(energyIni, distance);
   }
   virtual double GetEnergy(double energyIni, double distance) const override;

   virtual std::vector<std::pair<double, double>>
   GetBraggCurve(double energy, double rangeStepSize = 0, double totalFractionELoss = 0.001) const override;

   /**
    * Setter of the catima projectile used for calculations.
    * @param[in] A Mass number of the projectile.
    * @param[in] Z Charge number of the projectile.
    * @param[in] massUma Mass of the projectile in umas.
    */
   void SetProjectile(double A, double Z, double massUma)
   {
      fProjectile = std::make_unique<catima::Projectile>(A, Z);
      fProjectileMassUma = massUma;
   }
   /**
    * Setter of the range step size used for calculations. By default it is set to 0.1mm.
    * @param[in] stepSize The step size used for ranges. It must be input in mm.
    */
   void SetRangeStepSize(double stepSize) { fRangeStepSize = stepSize; }
};

} // namespace AtTools

#endif
