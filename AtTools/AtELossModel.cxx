#include "AtELossModel.h"

#include <stdexcept>
namespace AtTools {

/**
 * Set the density of the material we are calculating energy losses for in mg/cm^3.
 * Likely not fully tested, but I want to keep it around to remind myself of it.
 */
void AtELossModel::SetDensity(double density)
{
   fDensity = density;
}

void AtELossModel::SetELossModelName(std::string name)
{
   fELossModelName = name;
}

void AtELossModel::SetPDGCode(std::string pdg)
{
   fPDGCode = pdg;
}

void AtELossModel::SetChargeNumber(int z)
{
   fZ = z;
}

void AtELossModel::SetAtomicMassNumber(int a)
{
   fA = a;
}

void AtELossModel::SetMassAmu(double mass)
{
   fMassAmu = mass;
}

double AtELossModel::GetDensity()
{
   return fDensity;
}

std::string AtELossModel::GetELossModelName()
{
   return fELossModelName;
}

std::string AtELossModel::GetPDGCode()
{
   return fPDGCode;
}

int AtELossModel::GetChargeNumber()
{
   return fZ;
}

int AtELossModel::GetAtomicMassNumber()
{
   return fA;
}

double AtELossModel::GetMassAmu()
{
   return fMassAmu;
}

std::vector<std::pair<double, double>>
AtELossModel::GetBraggCurve(double energy, double rangeStepSize, double totalFractionELoss, double minRange) const
{
   std::vector<std::pair<double, double>> braggCurve;

   double remainingEnergy{energy};
   double range{};
   while (remainingEnergy / energy > totalFractionELoss) {
      remainingEnergy = GetEnergy(energy, range);
      double dEdx = GetdEdx(remainingEnergy);
      braggCurve.push_back(std::make_pair(dEdx, range));
      range += rangeStepSize;
   }

   if (!minRange)
      return braggCurve;

   while (range < minRange - rangeStepSize) {
      braggCurve.push_back(std::make_pair(0, range));
      range += rangeStepSize;
   }

   return braggCurve;
}

std::vector<std::pair<double, double>> AtELossModel::GetIntegratedELoss(double energy, double binSize, int valuesPerBin, double totalFractionELoss, double minRange) const
{
   auto braggCurve = GetBraggCurve(energy, binSize / valuesPerBin, totalFractionELoss, minRange);

   std::vector<std::pair<double, double>> integratedELoss;
   double currentELoss{};

   for (int i = 0; i < braggCurve.size(); i++) {
      auto currentPair = braggCurve[i];
      if ((i + 1) % valuesPerBin == 0) {
         int currentBinNum = integratedELoss.size();
         integratedELoss.push_back(std::make_pair(currentELoss, binSize * (1 / 2. + currentBinNum)));
         currentELoss = 0;
      }
      currentELoss += currentPair.first * binSize / valuesPerBin;
   }

   // If size of dE/dx is not multiple of the number of values per bin, the last ELoss was not saved. In that case we store it before returning the vector.
   if (braggCurve.size() % valuesPerBin != 0) {
      int currentBinNum = integratedELoss.size();
      integratedELoss.push_back(std::make_pair(currentELoss, binSize * (1 / 2. + currentBinNum)));
   }

   return integratedELoss;
}

} // namespace AtTools
