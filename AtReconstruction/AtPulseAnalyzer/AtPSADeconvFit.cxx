#include "AtPSADeconvFit.h"

#include "AtContainerManip.h"
#include "AtDataManip.h"
#include "AtDigiPar.h"

#include <FairLogger.h> // for Logger, LOG
#include <FairParSet.h> // for FairParSet
#include <FairRun.h>
#include <FairRuntimeDb.h>

#include <Math/WrappedParamFunction.h>
#include <TH1.h>     // for TH1D
#include <TMath.h>   // for Pi
#include <TString.h> // for TString

#include <HFitInterface.h>

#include <Fit/BinData.h>
#include <Fit/Chi2FCN.h>
#include <Fit/DataOptions.h> // for DataOptions
#include <Fit/DataRange.h>   // for DataRange
#include <algorithm>         // for max_element
#include <cmath>             // for sqrt
#include <functional>        // for hash
#include <iterator>          // for begin, distance, end
#include <memory>            // for allocator, unique_ptr
#include <thread>
thread_local std::unique_ptr<TH1F> AtPSADeconvFit::fHist = nullptr;

void AtPSADeconvFit::Init()
{
   AtPSADeconv::Init();
   auto fPar = dynamic_cast<AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   fDiffLong = fPar->GetCoefDiffusionLong();
}

AtPSADeconv::HitData AtPSADeconvFit::getZandQ(const AtPad::trace &charge)
{
   // Get initial guess for hit. Z loc is max and std dev is estimated from diffusion
   auto maxTB = std::max_element(begin(charge), end(charge));
   auto zTB = static_cast<double>(std::distance(begin(charge), maxTB));

   auto zPos = CalculateZGeo(zTB);               // [mm]
   auto driftTime = zPos / fDriftVelocity / 10.; // [us] drift velocity is cm/us
   if (driftTime < 0)
      driftTime = 0;
   auto longDiff = std::sqrt(2 * fDiffLong * driftTime); // [cm] longitudal diff sigma
   auto sigTime = longDiff / fDriftVelocity;             // [us] longitudal diff sigma
   auto sigTB = sigTime / fTBTime * 1000.;               // [TB] sigTime is us, TBTime is ns.
   if (sigTB < 0.1)
      sigTB = 0.1;
   LOG(debug) << "zTB: " << zTB << " zTime " << driftTime << " sigTB: " << sigTB << " Amp: " << *maxTB;

   if (*maxTB < getThreshold() || zTB < 20 || zTB > 500) {
      LOG(debug) << "Skipping pad: " << *maxTB << " below threshold or " << zTB
                 << " outside initial guess for hit TB is outside valid range (20,500).";
      return {};
   }

   // Create a historgram to fit and fit to range mean +- 4 sigma.
   auto id = std::hash<std::thread::id>{}(std::this_thread::get_id());
   if (fHist)
      ContainerManip::SetHistFromData(*fHist, charge);
   else
      fHist = ContainerManip::CreateHistFromData<TH1F>(TString::Format("%lu", id).Data(), charge);

   // Add an addition +-2 for when we are close to the pad plane and diffusion is small
   auto fitRange = 3 * sigTB + 2;
   if (fitRange < 3)
      fitRange = 3;

   // Fit without graphics. Start with the max as the height, position and a sigma set by diffusion.
   double par[3] = {*maxTB, zTB, sigTB};
   if (!FitGaussian(*fHist, zTB - fitRange, zTB + fitRange, par)) {
      LOG(info) << "Fit did not converge using initial conditions:"
                << " mean: " << zTB << " sig:" << sigTB << " max:" << *maxTB;
      return {};
   }

   auto amp = par[0];
   auto z = par[1];
   auto sig = par[2];

   auto Q = amp * sig * std::sqrt(2 * TMath::Pi());
   LOG(debug) << "Initial: " << *maxTB << " " << zTB << " " << sigTB;
   LOG(debug) << "Fit: " << amp << " " << z << " " << sig;

   return {{z, sig * sig, Q, 0}};
}

bool AtPSADeconvFit::FitGaussian(const TH1F &hist, double xmin, double xmax, double (&par)[3])
{
   // Create the data to fit
   ROOT::Fit::DataOptions opt;
   ROOT::Fit::DataRange range(xmin, xmax);
   ROOT::Fit::BinData d(opt, range);
   ROOT::Fit::FillData(d, &hist);

   // A gaussian with parameters (amplitude, mean, sigma), and the chi2 of it against the data
   auto gauss = [](const double *x, const double *p) {
      auto u = (x[0] - p[1]) / p[2];
      return p[0] * std::exp(-0.5 * u * u);
   };
   ROOT::Math::WrappedParamFunction<decltype(gauss)> func(gauss, 1, 3, par);
   ROOT::Fit::Chi2FCN<ROOT::Math::IMultiGenFunction> chi2(d, func);

   return AtTools::MinimizeMigrad(chi2, par);
}
