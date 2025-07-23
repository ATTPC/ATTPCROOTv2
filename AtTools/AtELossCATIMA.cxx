#include "AtELossCATIMA.h"

#include <FairLogger.h>
namespace AtTools {

AtELossCATIMA::AtELossCATIMA(double density, std::vector<std::tuple<int, int, int>> materialComponents)
   : AtELossModel(density)
{
   SetMaterial(materialComponents);
}

double AtELossCATIMA::GetdEdx(double energy) const
{
   if (fProjectile == nullptr) {
      LOG(warning)
         << " Warning in AtTools::AtELossCATIMA::GetdEdx : The projectile was not set! GetdEdx will return 0!";
      return 0;
   }

   if (fProjectileMassAmu <= 0) {
      LOG(error) << " Error in AtTools::AtELossCATIMA::GetdEdx : The projectile's mass in umas can not be <= 0! "
                    "GetdEdx will return 0!";
      return 0;
   }

   catima::Result result = catima::calculate(*fProjectile, *fMaterial, energy / fProjectileMassAmu);
   double dEdx = result.dEdxi * fDensity; // MeV/cm
   return dEdx / 10.0;                    // convert to MeV/mm
}

double AtELossCATIMA::GetRange(double energyIni, double energyFin) const
{
   if (energyFin < 0) {
      LOG(warning) << " Warning in AtTools::AtELossCATIMA::GetRange : The final energy was set to a negative value! "
                      "Setting energyFin to 0!";
      energyFin = 0;
   }

   if (energyFin == 0) {
      fProjectile->T = energyIni / fProjectileMassAmu;
      return catima::range(*fProjectile, *fMaterial) / fDensity * 10.;
   }

   double remainingEnergy{energyIni};
   double range{0};
   while (remainingEnergy > energyFin) {
      catima::Result result = catima::calculate(*fProjectile, *fMaterial, remainingEnergy / fProjectileMassAmu);
      double dEdx = result.dEdxi * fDensity;
      double DE = dEdx * fRangeStepSize / 10.;

      if (remainingEnergy - energyFin > DE) {
         range += fRangeStepSize;
         remainingEnergy -= DE;
      } else {
         range += (remainingEnergy - energyFin) / dEdx * 10.;
         remainingEnergy = energyFin;
         break;
      }
   }
   return range;
}

double AtELossCATIMA::GetEnergy(double energyIni, double distance) const
{
   double remainingEnergy{energyIni};
   double range{0};
   while (range < distance) {
      catima::Result result = catima::calculate(*fProjectile, *fMaterial, remainingEnergy / fProjectileMassAmu);
      double dEdx = result.dEdxi * fDensity;
      double DE{};

      if (range + fRangeStepSize < distance) {
         DE = dEdx * fRangeStepSize / 10.;
         range += fRangeStepSize;
         remainingEnergy -= DE;
      } else {
         DE = dEdx * (distance - range) / 10.;
         range = distance;
         remainingEnergy -= DE;
         break;
      }
   }

   return remainingEnergy;
}
double AtELossCATIMA::GetRangeVariance(double energy) const
{
   if (fProjectile == nullptr || fMaterial == nullptr) {
      LOG(error) << "Projectile or material not set. Range variance is 0.";
      return 0;
   }
   auto range_var =
      catima::range_variance(*fProjectile, energy / fProjectileMassAmu, *fMaterial); // range var in (g/cm^2)^2
   LOG(debug) << "Range variance in (g/cm^2)^2: " << range_var << " for energy: " << energy / fProjectileMassAmu
              << " MeV/u";
   range_var /= fDensity * fDensity; // convert to (cm)^2
   LOG(debug) << "Range variance in (cm)^2: " << range_var << " for energy: " << energy / fProjectileMassAmu
              << " MeV/u";
   return range_var * 100; // convert to mm^2
}

double AtELossCATIMA::GetElossStraggling(double energyIni, double energyFin) const
{
   if (fProjectile == nullptr || fMaterial == nullptr) {
      LOG(error) << "Projectile or material not set.";
      return 0;
   }
   if (energyFin > energyIni) {
      LOG(error) << "Final energy must be less than initial energy!";
      return 0;
   }
   auto energy_strag = catima::energy_straggling_from_E(*fProjectile, energyIni / fProjectileMassAmu,
                                                        energyFin / fProjectileMassAmu, *fMaterial);
   return energy_strag;
}
double AtELossCATIMA::GetdEdxStraggling(double energyIni, double energyFin) const
{
   if (fProjectile == nullptr || fMaterial == nullptr) {
      LOG(error) << "Projectile or material not set. dEdx straggling is 0.";
      return 0;
   }
   auto dedx_min = GetdEdx(energyIni);
   auto dedx_max = GetdEdx(energyFin);
   if (std::abs(dedx_min - dedx_max) / dedx_min > 0.01) {
      LOG(warning) << "From " << energyIni << " to " << energyFin
                   << " MeV the dEdx is not constant. dEdx straggling calculation is unreliable.";
   }
   auto dE_st = GetElossStraggling(energyIni, energyFin);
   auto factor = dE_st / (energyIni - energyFin);
   return factor * dedx_min;
}

std::vector<std::pair<double, double>>
AtELossCATIMA::GetBraggCurve(double energy, double rangeStepSize, double totalFractionELoss) const
{
   if (rangeStepSize == 0)
      return GetBraggCurve(energy, fRangeStepSize, totalFractionELoss);

   std::vector<std::pair<double, double>> braggCurve;

   double remainingEnergy{energy};
   double range{};
   while (remainingEnergy / energy > totalFractionELoss) {

      catima::Result result = catima::calculate(*fProjectile, *fMaterial, remainingEnergy / fProjectileMassAmu);
      double dEdx = result.dEdxi * fDensity;
      braggCurve.push_back(std::make_pair(dEdx, range));

      double DE = dEdx * rangeStepSize / 10.;
      if (DE > remainingEnergy)
         break;

      remainingEnergy -= DE;
      range += rangeStepSize;
   }

   return braggCurve;
}

void AtELossCATIMA::SetMaterial(std::vector<std::tuple<int, int, int>> materialComponents)
{
   fMaterial = std::make_unique<catima::Material>();
   for (const auto &materialComponent : materialComponents) {
      fMaterial->add_element(std::get<0>(materialComponent), std::get<1>(materialComponent),
                             std::get<2>(materialComponent));
   }
}

} // namespace AtTools