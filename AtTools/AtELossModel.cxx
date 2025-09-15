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
AtELossModel::GetBraggCurve(double energy, double rangeStepSize, double totalFractionELoss) const
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

   return braggCurve;
}

} // namespace AtTools
