#ifndef __CLING__
#include "../../build/include/AtCSVReader.h"
#endif

#include <TString.h>

#include <fstream>
#include <iostream>
class E12014SC {
private:
   double fLambda;

public:
   double GetLambda() const { return fLambda; }

   E12014SC(int nsclRunNumber)
   {
      TString fileName = TString::Format("/mnt/analysis/e12014/home/scalers/run%04d.csv", nsclRunNumber);
      std::ifstream scalar(fileName.Data());
      if (!scalar.is_open())
         throw std::invalid_argument("Could not open file " + fileName);
      double beamRate = 0;
      for (auto &row : CSVRange<std::string>(scalar)) {
         if (row[0] == "IC_OR")
            beamRate = std::stod(row[2]);
      }
      fLambda = beamRate * 3e-12 + 4e-9;
      std::cout << "Setting lambda to " << fLambda << " for " << beamRate << " pps." << std::endl;
   }

   double operator()(double rho, double z)
   {
      constexpr double rBeam = 2.0 / 100;    // in *m*
      constexpr double eps = 8.85418782E-12; // SI
      constexpr double pi = 3.14159265358979;
      constexpr double eps2pi = 2 * pi * eps;
      rho /= 100.; // Convert units from cm to m
      z /= 100.;   // Convert units from cm to m

      double field;
      if (rho > rBeam)
         field = fLambda / eps2pi / rho * (z / 1.); // v/m
      else
         // field = lambda / eps2pi / rBeam / rBeam * rho * (z/1.); // v/m
         field = 0;
      return field / 100.; // V/cm
   }
};
double EField(double rho, double z)
{
   double lambda = 1.54e-8;               // SI
   constexpr double rBeam = 2.0 / 100;    // in *m*
   constexpr double eps = 8.85418782E-12; // SI
   constexpr double pi = 3.14159265358979;
   constexpr double eps2pi = 2 * pi * eps;
   rho /= 100.; // Convert units from cm to m
   z /= 100.;   // Convert units from cm to m

   double field;
   if (rho > rBeam)
      field = lambda / eps2pi / rho * (z / 1.); // v/m
   else
      // field = lambda / eps2pi / rBeam / rBeam * rho * (z/1.); // v/m
      field = 0;
   return field / 100.; // V/cm
}
