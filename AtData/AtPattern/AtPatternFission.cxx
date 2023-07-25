#include "AtPatternFission.h"

#include "AtPattern.h" // for AtPatterns

#include <FairLogger.h> // for Logger, LOG

#include <Math/Functor.h>
#include <TString.h>

#include <Fit/FitConfig.h> // for FitConfig
#include <Fit/FitResult.h> // for FitResult
#include <Fit/Fitter.h>
#include <Fit/ParameterSettings.h> // for ParameterSettings
#include <algorithm>               // for max
#include <array>                   // for array
#include <memory>                  // for allocator, allocator_traits<>::va...

using namespace AtPatterns;

ClassImp(AtPatternFission);

void AtPatternFission::FitPattern(const std::vector<XYZPoint> &points, const std::vector<double> &charge)
{
   // Based on the example from ROOT documentation https://root.cern.ch/doc/master/line3Dfit_8C_source.html

   bool weighted = points.size() == charge.size();
   LOG(debug) << "Fitting with" << (weighted ? " " : "out ") << "charge weighting";

   // This functor is what we are minimizing. It takes in the model parameters and defines an example
   // of this pattern based on the model parameters. It then loops through every hit associated with
   // pattern and calculates the chi2.
   auto func = [&points, &charge, weighted, this](const double *par) {
      AtPatternY pat;
      pat.DefinePattern(std::vector<double>(par, par + 12));
      std::array<double, 3> chi2 = {0, 0, 0};
      std::array<double, 3> qTot = {0, 0, 0};

      for (int i = 0; i < points.size(); ++i) {
         auto q = weighted ? charge[i] : 1;
         auto pointAss = this->GetPointAssignment(points[i]);

         // We can skip including points if they were originally FF points, or they were beam points
         // and are now FF points and are too close to the center of the detector.
         bool origFF = pointAss < 2;
         origFF |= (pointAss == 2 && pat.GetPointAssignment(points[i]) < 2);

         if (origFF && points[i].Rho() < 20)
            continue;
         LOG(debug) << q << " " << pat.DistanceToPattern(points[i]);
         chi2[pointAss] += pat.DistanceToPattern(points[i]) * pat.DistanceToPattern(points[i]) * q;
         qTot[pointAss] += q;
      }

      double retVal = 0;

      // Only include the chi2 for the two fission fragments
      for (int i = 0; i < 2; ++i) {
         LOG(debug) << i << ": " << chi2[i] << "/" << qTot[i] << " = " << chi2[i] / qTot[i];
         if (qTot[i] != 0)
            retVal += chi2[i] / qTot[i];
      }
      LOG(debug) << "Obj: " << retVal;
      return retVal;
   };

   auto functor = ROOT::Math::Functor(func, 12);

   ROOT::Fit::Fitter fitter;
   auto iniPar = GetPatternPar();

   LOG(debug) << "Initial parameters";
   for (int i = 0; i < iniPar.size(); ++i)
      LOG(debug) << Form("Par_%d", i) << "\t = " << iniPar[i];

   fitter.SetFCN(functor, iniPar.data());

   // Constrain the Z direction to be the same
   for (int i = 0; i < 3; ++i)
      fitter.Config().ParSettings(5 + i * 3).Fix();

   for (int i = 0; i < 12; ++i)
      fitter.Config().ParSettings(i).SetStepSize(.01);

   bool ok = fitter.FitFCN();
   if (!ok) {
      LOG(error) << "Failed to fit the pattern, using result of SAC";
      DefinePattern(iniPar);
      return;
   }

   auto &result = fitter.Result();
   DefinePattern(result.Parameters());
   fChi2 = result.MinFcnValue();
   fNFree = points.size() - result.NPar();
}
