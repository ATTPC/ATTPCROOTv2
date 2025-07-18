#include "AtELossModel.h"

#include <stdexcept>
namespace AtTools {

/**
 * Set the density of the material we are calculating energy losses for in mg/cm^3.
 * Likely not fully tested, but I want to keep it around to remind myself of it.
 */
void AtELossModel::SetDensity(double density)
{
   if (fDensityIni == 0)
      throw std::invalid_argument("Cannot set the density if the density of in model is not known");

   fDensity = density;
   fdEdxScale = fDensity / fDensityIni;
}

std::vector<std::pair<double, double>> AtELossModel::GetBraggCurve(double energy, double rangeStepSize) const
{
   std::vector<std::pair<double, double>> braggCurve;

   double remainingEnergy{energy};
   double range{};
   while (remainingEnergy > 0) {
      remainingEnergy = GetEnergy(remainingEnergy, range);
      double dEdx = GetdEdx(remainingEnergy);
      braggCurve.push_back(std::make_pair(dEdx, range));
      range += rangeStepSize;
   }

   return braggCurve;
}

} // namespace AtTools
