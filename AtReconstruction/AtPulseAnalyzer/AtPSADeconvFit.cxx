#include "AtPSADeconvFit.h"

#include "AtContainerManip.h"
#include "AtDigiPar.h"

#include <FairLogger.h> // for Logger, LOG
#include <FairParSet.h> // for FairParSet
#include <FairRun.h>
#include <FairRuntimeDb.h>

#include <TF1.h>
#include <TFitResult.h>
#include <TFitResultPtr.h> // for TFitResultPtr
#include <TH1.h>           // for TH1D
#include <TMath.h>         // for Pi

#include <algorithm> // for max_element
#include <cmath>     // for sqrt
#include <iterator>  // for begin, distance, end
#include <memory>    // for allocator, unique_ptr

AtPSADeconv::HitData AtPSADeconvFit::getZandQ(const AtPad::trace &charge)
{
   // Get initial guess for hit. Z loc is max and std dev is estimated from diffusion
   auto maxTB = std::max_element(begin(charge), end(charge));
   auto fPar = dynamic_cast<AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));

   auto zTB = static_cast<double>(std::distance(begin(charge), maxTB));
   auto diffCoeff = fPar->GetCoefDiffusionLong();

   auto zPos = CalculateZGeo(zTB);                       // [mm]
   auto driftTime = zPos / fDriftVelocity / 10.;         // [us] drift velocity is cm/us
   auto longDiff = std::sqrt(2 * diffCoeff * driftTime); // [cm] longitudal diff sigma
   auto sigTime = longDiff / fDriftVelocity;             // [us] longitudal diff sigma
   auto sigTB = sigTime / fTBTime * 1000.;               // [TB] sigTime is us, TBTime is ns.

   LOG(debug) << "zTB: " << zTB << " zTime " << driftTime << " sigTB: " << sigTB << " Amp: " << *maxTB;

   if (*maxTB < getThreshold() || zTB < 20 || zTB > 500) {
      LOG(debug) << "Skipping pad: " << *maxTB << " below threshold or " << zTB
                 << " outside initial guess for hit TB is outside valid range (20,500).";
      return {};
   }

   // Create a historgram to fit and fit to range mean +- 4 sigma.
   auto hist = ContainerManip::CreateHistFromData("Qreco_fit", charge);

   // Add an addition +-2 for when we are close to the pad plane and diffusion is small
   auto fitRange = 3 * sigTB + 2;
   TF1 gauss("fitGauss", "gaus(0)", zTB - fitRange, zTB + fitRange);
   gauss.SetParameter(0, *maxTB); // Set initial height of gaussian
   gauss.SetParameter(1, zTB);    // Set initial position of gaussian
   gauss.SetParameter(2, sigTB);  // Set initial sigma of gaussian

   // Fit without graphics and saving everything in the result ptr
   auto resultPtr = hist->Fit(&gauss, "SQNR");
   if (resultPtr.Get() == nullptr) {
      LOG(info) << "Null fit for pad using mean and deviation of trace";
      return {};
   }

   auto amp = resultPtr->GetParams()[0];
   auto z = resultPtr->GetParams()[1];
   auto sig = resultPtr->GetParams()[2];

   auto Q = amp * sig * std::sqrt(2 * TMath::Pi());
   LOG(debug) << "Initial: " << zTB << " " << sigTB << " " << *maxTB;
   LOG(debug) << "Fit: " << z << " " << sig << " " << amp;
   // LOG(info) << "Q: " << Q;

   /// TODO: Error in charge?

   return {{z, sig * sig, Q, 0}};
}
