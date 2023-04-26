#include "AtPSADeconvFit.h"

#include "AtContainerManip.h"
#include "AtDigiPar.h"

#include <FairLogger.h> // for Logger, LOG
#include <FairParSet.h> // for FairParSet
#include <FairRun.h>
#include <FairRuntimeDb.h>

#include <Math/WrappedMultiTF1.h>
#include <TF1.h>
#include <TH1.h>     // for TH1D
#include <TMath.h>   // for Pi
#include <TString.h> // for TString

#include <HFitInterface.h>

#include <Fit/BinData.h>
#include <Fit/DataOptions.h> // for DataOptions
#include <Fit/DataRange.h>   // for DataRange
#include <Fit/FitConfig.h>   // for FitConfig
#include <Fit/Fitter.h>
#include <algorithm>  // for max_element
#include <cmath>      // for sqrt
#include <functional> // for hash
#include <iterator>   // for begin, distance, end
#include <memory>     // for allocator, unique_ptr
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

   TF1 gauss(TString::Format("fitGauss%lu", id), "gaus(0)", zTB - fitRange, zTB + fitRange, TF1::EAddToList::kNo);
   gauss.SetParameter(0, *maxTB); // Set initial height of gaussian
   gauss.SetParameter(1, zTB);    // Set initial position of gaussian
   gauss.SetParameter(2, sigTB);  // Set initial sigma of gaussian

   // Fit without graphics and saving everything in the result ptr
   // auto resultPtr = hist->Fit(&gauss, "SQNR");
   auto result = FitHistorgramParallel(*fHist, gauss);
   if (!result.IsValid()) {
      LOG(info) << "Fit did not converge using initial conditions:"
                << " mean: " << zTB << " sig:" << sigTB << " max:" << *maxTB;
      return {};
   }

   auto amp = result.GetParams()[0];
   auto z = result.GetParams()[1];
   auto sig = result.GetParams()[2];

   auto Q = amp * sig * std::sqrt(2 * TMath::Pi());
   LOG(debug) << "Initial: " << *maxTB << " " << zTB << " " << sigTB;
   LOG(debug) << "Fit: " << amp << " " << z << " " << sig;

   return {{z, sig * sig, Q, 0}};
}

const ROOT::Fit::FitResult AtPSADeconvFit::FitHistorgramParallel(TH1F &hist, TF1 &func)
{
   // Create the data to fit
   double xmin = 0, xmax = 0;
   func.GetRange(xmin, xmax);
   ROOT::Fit::DataOptions opt;

   ROOT::Fit::DataRange range(xmin, xmax);
   ROOT::Fit::BinData d(opt, range);
   ROOT::Fit::FillData(d, &hist);

   // Create the function to fit (just a wrapper)
   ROOT::Math::WrappedMultiTF1 wf(func);

   ROOT::Fit::Fitter fFitter;
   fFitter.Config().SetMinimizer("Minuit2");
   fFitter.SetFunction(wf, false);

   bool goodFit = fFitter.Fit(d);
   if (goodFit)
      return fFitter.Result();
   else
      return {};
}
