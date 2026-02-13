#include "AtELossTable.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtStringManip.h"

#include <FairLogger.h>

#include <algorithm> // for max
#include <cmath>     // for fabs
#include <iostream>

namespace AtTools {

void AtELossTable::SetDensity(double density)
{
   if (fDensity == 0)
      throw std::invalid_argument("Cannot set the density of a E loss table if the current density is not known");

   auto fDensityIni = fDensity; // store initial density
   AtELossModel::SetDensity(density);
   LOG(info) << "Scaling energy loss table to " << density << " g/cm^3 from " << fDensity << " g/cm^3";
   auto x = fdXdE.get_x();
   auto y = fdXdE.get_y();
   auto r_var = fRangeVariance.get_y();
   auto fScaling = density / fDensityIni;
   for (auto &elem : y) {
      elem = fScaling / elem; // scale dx/de by the density
   }
   for (auto &elem : r_var) {
      LOG(debug) << "Scaling range variance by " << fScaling * fScaling;
      elem = elem / (fScaling * fScaling); // scale range variance by the density
   }
   LoadTable(x, y);
   LoadRangeVariance(x, r_var);
}

void AtELossTable::LoadTable(const std::vector<double> &energy, const std::vector<double> &dEdX)
{

   if (energy.size() > 0)
      LOG(info) << "Loading dE/dX from " << energy.front() << " MeV to " << energy.back() << " MeV ";
   else
      LOG(error) << "Passed table was empty!";

   std::vector<double> dXdE;
   for (auto &elem : dEdX)
      dXdE.push_back(1 / elem);
   fdXdE = tk::spline(energy, dXdE);
}

AtELossTable::AtELossTable(const std::vector<double> &energy, const std::vector<double> &dEdX, double density)
   : AtELossModel(density)
{
   LoadTable(energy, dEdX);
}

double AtELossTable::GetdEdx(double energy) const
{
   double dedx = 1 / fdXdE(energy);
   return dedx;
}

double AtELossTable::GetRange(double energyIni, double energyFin) const
{
   if (energyIni == energyFin)
      return 0;
   if (energyFin < fdXdE.get_x_min()) {
      LOG(debug) << "Attemting to integrate energy to " << energyFin << " when min energy in table is "
                 << fdXdE.get_x_min();
      energyFin = fdXdE.get_x_min();
   }
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
         LOG(info) << "Energy converged in " << i + 1 << " iterations.";
         return guessEnergy;
      }
      LOG(debug) << "guessE: " << guessEnergy << " d: " << distance << " R: " << range
                 << " dEdX: " << GetdEdx(guessEnergy) << " dEnergy: " << GetdEdx(guessEnergy) * (distance - range);
      // Update guess energy using what is essentially Newton's method
      guessEnergy += GetdEdx(guessEnergy) * (range - distance);
      if (guessEnergy < fdXdE.get_x_min() * 1.01) {
         guessEnergy = fdXdE.get_x_min();
         return guessEnergy;
      }
   }

   LOG(error) << "Energy calculation (" << energyIni << " MeV through " << distance << " mm) failed to converge in "
              << maxIt << " iterations!";
   return -1;
}

double AtELossTable::GetRangeVariance(double energy) const
{
   if (fRangeVariance.get_size() == 0 || fRangeVariance.get_x_min() > energy || fRangeVariance.get_x_max() < energy) {
      LOG(error) << "Energy " << energy << " MeV is out of range for range variance table!";
      return 0;
   }
   // doubling density halves straggling.
   auto variance = fRangeVariance(energy); // with ini density
   return variance;
}

double AtELossTable::GetElossStraggling(double energyIni, double energyFin) const
{
   return GetdEdx(energyIni) * std::sqrt(GetRangeVariance(energyIni) - GetRangeVariance(energyFin));
}
double AtELossTable::GetdEdxStraggling(double energyIni, double energyFin) const
{
   double factor =
      std::sqrt(GetRangeVariance(energyIni) - GetRangeVariance(energyFin)) / GetRange(energyIni, energyFin);
   return GetdEdx(energyIni) * factor;
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

void AtELossTable::LoadSrimTable(std::string fileName)
{
   std::ifstream file(fileName);
   if (!file.is_open())
      LOG(fatal) << "Failed to open SRIM file " << fileName;

   bool atConversion = false;
   double conversion = 0;
   std::vector<double> energy;
   std::vector<double> dEdX;
   std::vector<double> rangeVar;

   while (!file.eof()) {
      // Get the current line
      std::string line;
      std::getline(file, line);
      auto tokens = SplitString(line, ' ');
      LOG(debug) << "Processing line " << line;

      // If this is the densityt line, grab it and continue
      if (tokens[0] == "Target" && tokens[1] == "Density") {
         LOG(info) << "Setting target density to: " << tokens[3] << " g/cm^3";
         fDensity = std::stod(tokens[3]);
         continue;
      }

      // Check if we've reached the conversion part of the table
      atConversion |= (tokens[0] == "Multiply");
      LOG(debug) << "At converion: " << atConversion;

      // If we're at the conversion stage, look for the correct conversion factor
      try {
         if (atConversion && tokens.at(1) == "MeV" && tokens.at(3) == "mm") {
            conversion = std::stod(tokens.at(0));
            LOG(info) << "Using conversion factor of " << conversion << " from " << tokens.at(0);
            break;
         }

         //  If this is part of the table, grab it and continue
         double en = std::stod(tokens.at(0)) * GetUnitConversion(tokens.at(1));
         double dedx = std::stod(tokens.at(2)) + std::stod(tokens.at(3));
         double range = std::stod(tokens.at(4)) * GetUnitConversion(tokens.at(5));
         double range_st = std::stod(tokens.at(6)) * GetUnitConversion(tokens.at(7));

         LOG(info) << "En: " << en << " dEdX " << dedx << " Range: " << range << " Range st: " << range_st;

         // If we made it this far then we have a valid table entry
         energy.push_back(en);
         dEdX.push_back(dedx);
         rangeVar.push_back(range_st * range_st);

         LOG(debug) << en << " " << dedx;

      } catch (...) {
      }
   } // end loop over file

   for (auto &dedx : dEdX) {
      dedx *= conversion;
   }

   for (int i = 0; i < dEdX.size(); ++i) {
      LOG(debug) << "Energy: " << energy[i] << " MeV dEdX: " << dEdX[i] << " MeV/mm";
   }

   LoadTable(energy, dEdX);
   LoadRangeVariance(energy, rangeVar);
}

void AtELossTable::LoadLiseTable(std::string fileName, double mass, double density, int column)
{
   std::ifstream file(fileName);
   if (!file.is_open())
      LOG(fatal) << "Failed to open LISE file " << fileName;

   std::vector<double> energy;
   std::vector<double> dEdX;
   fDensity = (fabs(density));

   while (!file.eof()) {
      // Get the current line
      std::string line;
      std::getline(file, line);
      auto tokens = SplitString(line, '\t');
      LOG(debug) << "Processing line " << line;

      try {
         //  If this is part of the table, grab it and continue
         double en = std::stod(tokens.at(0 + column * 2)) * mass;
         double dedx = std::stod(tokens.at(1 + column * 2));

         if (density > 0) // dEdX is in units MeV/(mg/cm^2)
            dedx *= (100 * density);
         else // dEdX is in units MeV/um
            dedx *= 1000;

         LOG(debug) << "En: " << en << " dEdX " << dedx;
         // If we made it this far then we have a valid table entry
         energy.push_back(en);
         dEdX.push_back(dedx);
      } catch (...) {
      }
   }

   LoadTable(energy, dEdX);
}

double AtELossTable::GetUnitConversion(const std::string &unit)
{
   if (unit == "eV")
      return 1e-6;
   if (unit == "keV")
      return 1e-3;
   if (unit == "MeV")
      return 1e-0;
   if (unit == "GeV")
      return 1e3;
   if (unit == "um")
      return 1e-3;
   if (unit == "mm")
      return 1e0;
   if (unit == "cm")
      return 1e1;
   if (unit == "m")
      return 1e3;
   return 0;
}
} // namespace AtTools
