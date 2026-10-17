#include "AtDataManip.h"

#include "AtDigiPar.h"

#include <FairLogger.h>
#include <FairParSet.h> // for FairParSet
#include <FairRun.h>
#include <FairRuntimeDb.h>

#include <Math/IFunction.h>
#include <TF1.h>
#include <TMath.h> // for Pi

#include <AtHit.h>

#include <Minuit2/Minuit2Minimizer.h>
#include <algorithm> // for copy_n
#include <cmath>     // for sqrt
#include <string>

/**
 * Assumes that window is at z = 0, and the electrons are drifting towards the pad plane at
 * the z locations specified in the parameter file.
 */
double AtTools::GetTB(double z, const AtDigiPar *fPar)
{
   if (fPar == nullptr)
      fPar = dynamic_cast<const AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   if (fPar == nullptr) {
      LOG(error) << "Could not find the digipar file!";
      return -1;
   }

   auto driftDistance = fPar->GetZPadPlane() - z;
   auto driftTB = fPar->GetTBEntrance() - GetDriftTB(driftDistance, fPar);
   LOG(debug) << driftDistance << " " << driftTB;

   return driftTB;
}

double AtTools::GetDriftTB(double d, const AtDigiPar *fPar)
{
   if (fPar == nullptr)
      fPar = dynamic_cast<const AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   if (fPar == nullptr) {
      LOG(error) << "Could not find the digipar file!";
      return -1;
   }
   auto driftTime = d / (fPar->GetDriftVelocity() * 10.);
   return driftTime / (fPar->GetTBTime() / 1000.);
}
std::unique_ptr<TF1> AtTools::GetHitFunction(const AtHit &hit, const AtDigiPar *fPar)
{
   if (hit.GetPositionSigma().Z() == 0) {
      LOG(error) << "Hits that are points (sig_z = 0) are not supported yet!";
      return nullptr;
   }

   // position in mm to TB.
   if (fPar == nullptr)
      fPar = dynamic_cast<const AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));

   if (fPar == nullptr) {
      LOG(error) << "Could not find the digipar file!";
      return nullptr;
   }

   // Create the function we are going to set and make sure it isn't added to the global list
   auto func = std::make_unique<TF1>("hitFunc", "gaus", 0, fPar->GetZPadPlane(), TF1::EAddToList::kNo);
   auto ampl = hit.GetCharge() / (hit.GetPositionSigma().Z() * std::sqrt(2 * TMath::Pi()));
   func->SetParameter(0, ampl);
   func->SetParameter(1, hit.GetPosition().Z());
   func->SetParameter(2, hit.GetPositionSigma().Z());

   return func;
}

std::optional<AtTools::GaussianTB> AtTools::GetHitGaussianTB(const AtHit &hit, const AtDigiPar *fPar)
{
   if (hit.GetPositionSigma().Z() == 0) {
      LOG(error) << "Hits that are points (sig_z = 0) are not supported yet!";
      return std::nullopt;
   }
   if (fPar == nullptr)
      fPar = dynamic_cast<const AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));

   auto sigma = GetDriftTB(hit.GetPositionSigma().Z(), fPar);
   return GaussianTB{hit.GetCharge() / (sigma * std::sqrt(2 * TMath::Pi())), GetTB(hit.GetPosition().Z(), fPar), sigma};
}

std::unique_ptr<TF1> AtTools::GetHitFunctionTB(const AtHit &hit, const AtDigiPar *fPar)
{
   auto gauss = GetHitGaussianTB(hit, fPar);
   if (gauss == std::nullopt)
      return nullptr;

   // Create the function we are going to set and make sure it isn't added to the global list
   auto func = std::make_unique<TF1>("hitFuncTB", "gaus", 0, 512, TF1::EAddToList::kNo);
   func->SetParameters(gauss->amp, gauss->mean, gauss->sigma);

   return func;
}

bool AtTools::MinimizeMigrad(const ROOT::Math::IMultiGenFunction &fcn, double *par, double *minValue)
{
   // ROOT::Fit::Fitter creates its minimizer through the plugin manager under ROOT's global lock, so threads making
   // many small fits queue on that lock. Build Minuit2 directly instead, with Fitter's step sizes and call limit.
   // Tolerance, strategy and print level are the ROOT::Math::MinimizerOptions defaults, as in Fitter.
   ROOT::Minuit2::Minuit2Minimizer minimizer(ROOT::Minuit2::kMigrad);
   minimizer.SetFunction(fcn);
   const int n = fcn.NDim();
   minimizer.SetMaxFunctionCalls(1000 + 100 * n + 5 * n * n);
   for (int i = 0; i < n; ++i)
      minimizer.SetVariable(i, "p" + std::to_string(i), par[i], par[i] != 0 ? 0.3 * std::fabs(par[i]) : 0.3);
   if (!minimizer.Minimize())
      return false;
   std::copy_n(minimizer.X(), n, par);
   if (minValue != nullptr)
      *minValue = minimizer.MinValue();
   return true;
}
