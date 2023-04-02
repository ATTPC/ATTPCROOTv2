#include "AtELossTable.h"

#include <FairLogger.h>

#include <iostream>

namespace AtTools {

AtELossTable::AtELossTable(const std::vector<double> &energy, const std::vector<double> &dEdX, double density)
   : AtELossModel(density)
{
   std::vector<double> dXdE;
   for (auto &elem : dEdX)
      dXdE.push_back(1 / elem);
   fdXdE = tk::spline(energy, dXdE);
}

double AtELossTable::GetdEdx(double energy) const
{
   double dedx = 1 / fdXdE(energy);
   return dedx * fdEdxScale;
}

double AtELossTable::GetRange(double energyIni, double energyFin) const
{
   if (energyIni == energyFin)
      return 0;

   return fdXdE.integrate(energyFin, energyIni);
}

double AtELossTable::GetEnergy(double energyIni, double distance) const
{
   if (distance == 0)
      return energyIni;
   if (energyIni < 1e-6 || GetRange(energyIni) < distance)
      return 0.;

   int maxIt = 100;

   double guessEnergy = energyIni - GetdEdx(energyIni) * distance;
   for (int i = 0; i < maxIt; ++i) {

      double range = GetRange(energyIni, guessEnergy);
      if (fabs(range - distance) < fDistErr) {
         LOG(debug) << "Energy converged in " << i + 1 << " iterations.";
         return guessEnergy;
      }
      LOG(debug) << "guessE: " << guessEnergy << " d: " << distance << " R: " << range
                 << " dEdX: " << GetdEdx(guessEnergy) << " dEnergy: " << GetdEdx(guessEnergy) * (distance - range);
      // Update guess energy using what is essentially Newton's method
      guessEnergy += GetdEdx(guessEnergy) * (range - distance);
   }

   LOG(error) << "Energy calculation failed to converge in " << maxIt << " iterations!";
   return -1;
}

double AtELossTable::GetEnergyOld(double energyIni, double distance) const
{
   if (distance == 0)
      return energyIni;
   if (energyIni < 1e-6)
      return 0.;

   double maxRange = GetRange(energyIni);

   if (distance > maxRange)
      return 0.;

   // Searcg for energyFinal where GetRange(energyIni, energyFinal) == distance

   double lowGuess = 0;
   double highGuess = energyIni;
   double guessEnergy = energyIni / 2.0;
   double range = GetRange(energyIni, guessEnergy);
   double distErr = 1.0e-4; // in mm
   int numIt = 1;

   while (fabs(range - distance) > distErr) {
      numIt++;
      // We went too far
      if (range > distance) {
         lowGuess = guessEnergy;
         guessEnergy = (lowGuess + highGuess) / 2.0;
      } else {
         // We didn't go far enough
         highGuess = guessEnergy;
         guessEnergy = (lowGuess + highGuess) / 2.0;
      }
      range = GetRange(energyIni, guessEnergy);
   }

   LOG(info) << "Energy converged in " << numIt << " iterations.";
   return guessEnergy;
}

} // namespace AtTools
