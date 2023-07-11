#include "AtPSADeconv.h"

#include "AtDigiPar.h"
#include "AtHit.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h" // for AtPadBase
#include "AtPadFFT.h"

#include <FairLogger.h>
#include <FairParSet.h> // for FairParSet
#include <FairRun.h>
#include <FairRuntimeDb.h>

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for XYZPoint
#include <Rtypes.h>          // for Int_t, Double_t
#include <TComplex.h>
#include <TVirtualFFT.h>

#include <cmath> // for sqrt
#include <numeric>
#include <stdexcept> // for runtime_error
#include <utility>   // for move, pair

using XYZPoint = ROOT::Math::XYZPoint;

AtPSADeconv::AtPSADeconv() : AtPSA()
{
   initFFTs();
};

AtPSADeconv::AtPSADeconv(const AtPSADeconv &r)
   : fEventResponse(r.fEventResponse), fResponse(r.fResponse), fFFT(nullptr), fFFTbackward(nullptr),
     fFilterOrder(r.fFilterOrder), fCutoffFreq(r.fCutoffFreq), fUseSimulatedCharge(r.fUseSimulatedCharge)
{
   initFFTs();
}

void AtPSADeconv::SetFilterOrder(int order)
{
   if (order % 2 != 0)
      LOG(error) << "We only support even order filters!";
   fFilterOrder = order / 2;
   initFilter();
}

void AtPSADeconv::SetCutoffFreq(int freq)
{
   fCutoffFreq = freq * freq;
   initFilter();
}

void AtPSADeconv::initFFTs()
{
   std::vector<Int_t> dimSize = {512};
   // Create a FFT object that we own ("K"), that will optimize the transform ("M"),
   // and is a forward transform from real data to complex ("R2C")
   fFFT = std::unique_ptr<TVirtualFFT>(TVirtualFFT::FFT(1, dimSize.data(), "R2C M K"));
   // Create a FFT object that we own ("K"), that will optimize the transform ("M"),
   // and is a backwards transform from complex to Reak ("C2R")
   fFFTbackward = std::unique_ptr<TVirtualFFT>(TVirtualFFT::FFT(1, dimSize.data(), "C2R M K"));
}

/**
 * @brief Get the AtPad describing the response of the electronics.
 *
 * If the pad is not in fEventResponse it will be added.
 */
AtPad &AtPSADeconv::GetResponse(int padNum)
{
   LOG(debug2) << "Getting pad " << padNum << " from response event.";
   auto pad = fEventResponse.GetPad(padNum);
   if (pad == nullptr)
      pad = createResponsePad(padNum);

   return *pad;
}

/**
 * @brief Get the fourier transform describing the response of the electronics.
 *
 * If the pad is not in fEventResponse it will be added, and if needed the fft will be calcualted and
 * added as an augment with the name "fft".
 */
const AtPadFFT &AtPSADeconv::GetResponseFFT(int padNum)
{
   auto &pad = GetResponse(padNum);
   auto fft = dynamic_cast<AtPadFFT *>(pad.GetAugment("fft"));

   if (fft == nullptr) {
      LOG(debug) << "Adding FFT to pad " << padNum;
      fFFT->SetPoints(pad.GetADC().data());
      fFFT->Transform();
      auto fftNew = std::make_unique<AtPadFFT>();
      fftNew->GetDataFromFFT(fFFT.get());
      fft = dynamic_cast<AtPadFFT *>(pad.AddAugment("fft", std::move(fftNew)));
   }

   return *fft;
}

/**
 * @brief Get the filter in fourier space describing the response of the electronics.
 *
 * If the pad is not in fEventResponse it will be added, and if needed the fft and filter will be
 * calculated and added as augments with the names "fft" and "filter", respectivley.
 */
const AtPadFFT &AtPSADeconv::GetResponseFilter(int padNum)
{
   auto &pad = GetResponse(padNum);
   auto &fft = GetResponseFFT(padNum);
   auto filter = dynamic_cast<AtPadFFT *>(pad.GetAugment("filter"));

   if (filter == nullptr) {
      LOG(debug) << "Adding filter to pad " << padNum;
      filter = dynamic_cast<AtPadFFT *>(pad.AddAugment("filter", std::make_unique<AtPadFFT>()));
      updateFilter(fft, filter);
   }
   return *filter;
}
void AtPSADeconv::updateFilter(const AtPadFFT &fft, AtPadFFT *filter)
{
   LOG(debug) << "Updating filter ";
   for (int i = 0; i < 512 / 2 + 1; ++i) {
      auto R = fft.GetPointComplex(i);
      auto filterVal = getFilterKernel(i) / R;

      LOG(debug2) << i << " " << TComplex::Abs(R) << " " << getFilterKernel(i) << " " << filterVal;
      filter->SetPointRe(i, filterVal.Re());
      filter->SetPointIm(i, filterVal.Im());
   }
}

AtPad *AtPSADeconv::createResponsePad(int padNum)
{
   LOG(debug) << "Creating response pad for " << padNum;
   auto fPar = dynamic_cast<AtDigiPar *>(FairRun::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   auto tbTime = fPar->GetTBTime() / 1000.;

   auto pad = fEventResponse.AddPad(padNum);
   LOG(debug) << "Filling response pad for " << padNum;
   for (int i = 0; i < 512; ++i) {
      auto time = (i + 0.5) * tbTime;
      pad->SetADC(i, fResponse(padNum, time));
   }
   return pad;
}

/**
 * @param[in] freq The frequency compnent
 * @return The kernel of the low pass filter as set at that frequency
 * @todo: Precompute once and save with the transformed response function
 */
double AtPSADeconv::getFilterKernel(int freq)
{
   if (fFilterOrder == 0)
      return 1;
   return 1.0 / (1.0 + std::pow(freq * freq / fCutoffFreq, fFilterOrder));
}

/**
 * @brief Update all "filter" augments in fEventResponse with the new parameters
 */
void AtPSADeconv::initFilter()
{
   // Loop through every existing filter and update it
   for (auto &pad : fEventResponse.GetPads()) {
      auto filter = dynamic_cast<AtPadFFT *>(pad->GetAugment("filter"));
      if (filter != nullptr)
         updateFilter(GetResponseFFT(pad->GetPadNum()), filter);
   }
}

// Assumes the passed pad already has its FFT calculated
AtPSADeconv::HitVector AtPSADeconv::AnalyzeFFTpad(AtPad &pad)
{
   LOG(debug) << "Analyzing pad " << pad.GetPadNum();
   auto padFFT = dynamic_cast<AtPadFFT *>(pad.GetAugment("fft"));
   auto recoFFT = dynamic_cast<AtPadFFT *>(pad.AddAugment("Qreco-fft", std::make_unique<AtPadFFT>()));
   LOG(debug) << "Getting response filter";
   const auto &respFFT = GetResponseFilter(pad.GetPadNum());
   LOG(debug) << "Got response filter";

   if (padFFT == nullptr)
      throw std::runtime_error("Missing FFT information in pad");

   // Fill the inverse FFT with the input charge
   for (int i = 0; i < 512 / 2 + 1; ++i) {
      auto a = padFFT->GetPointComplex(i);
      auto b = respFFT.GetPointComplex(i);
      LOG(debug2) << i << " " << a << " " << b << " " << a * b;
      auto z = a * b;

      recoFFT->SetPoint(i, z);
      z /= 512;
      fFFTbackward->SetPointComplex(i, z);
   }
   fFFTbackward->Transform();

   // Get baseline from the charge
   double baseline = std::accumulate(fFFTbackward->GetPointsReal(), fFFTbackward->GetPointsReal() + 20, 0.0);
   baseline /= 20;

   // Create the charge padcentercoord
   auto charge = std::make_unique<AtPadArray>();
   // Fill the charge pad
   for (int i = 0; i < 512; ++i)
      charge->SetArray(i, fFFTbackward->GetPointReal(i) - baseline);

   pad.AddAugment("Qreco", std::move(charge));

   return chargeToHits(pad, "Qreco");
}

AtPSADeconv::HitVector AtPSADeconv::AnalyzePad(AtPad *pad)
{
   // If this pad has simulated charge, use that instead
   if (fUseSimulatedCharge && pad->GetAugment<AtPadArray>("Q") != nullptr)
      return chargeToHits(*pad, "Q");

   // If this pad already contains FFT information, then just use it as is.
   if (dynamic_cast<AtPadFFT *>(pad->GetAugment("fft")) != nullptr)
      return AnalyzeFFTpad(*pad);

   // Add FFT data to this pad
   fFFT->SetPoints(pad->GetADC().data());
   fFFT->Transform();
   pad->AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));

   // Now process the pad with its fourier transform
   return AnalyzeFFTpad(*pad);
}

AtPSADeconv::HitVector AtPSADeconv::chargeToHits(AtPad &pad, std::string qName)
{

   HitVector ret;
   auto charge = dynamic_cast<AtPadArray *>(pad.GetAugment(qName));

   LOG(debug) << "PadNum: " << pad.GetPadNum();
   auto hitVec = getZandQ(charge->GetArray());

   for (auto &ZandQ : hitVec) {
      XYZPoint pos(pad.GetPadCoord().X(), pad.GetPadCoord().Y(), CalculateZGeo(ZandQ.z));

      auto posVarXY = getXYhitVariance();
      auto posVarZ = getZhitVariance(0, ZandQ.zVar);

      LOG(debug) << "Z(tb): " << ZandQ.z << " +- " << std::sqrt(ZandQ.zVar);
      LOG(debug) << "Z(mm): " << pos.Z() << " +- " << std::sqrt(posVarZ);

      auto hit = std::make_unique<AtHit>(pad.GetPadNum(), pos, ZandQ.q);
      hit->SetPositionVariance({posVarXY.first, posVarXY.second, posVarZ});
      hit->SetChargeVariance(ZandQ.qVar);

      ret.push_back(std::move(hit));
   }

   return ret;
}

AtPSADeconv::HitData AtPSADeconv::getZandQ(const AtPad::trace &charge)
{
   // Get the mean time and total charge
   double q = std::accumulate(charge.begin(), charge.end(), 0.0);
   double z = 0;
   for (int i = 0; i < charge.size(); ++i)
      z += i * charge[i];
   z /= q;

   // Get the variance of the time
   double zVar = 0;
   for (int i = 0; i < charge.size(); ++i) {
      zVar += charge[i] * (i - z) * (i - z);
   }
   zVar /= (q - 1);

   // Get the variance of the charge
   double qVar = 0;

   return {{z, zVar, q, qVar}}; // Vector containing a single ZHitData struct
}

double AtPSADeconv::getZhitVariance(double zLoc, double zLocVar) const
{
   // zLocVar is in TB^2
   double time = zLocVar * fTBTime * fTBTime;         // Get variance in ns^2
   time /= 1000 * 1000;                               // Get variance in us^2
   auto pos = time * fDriftVelocity * fDriftVelocity; // Get variance in cm
   pos *= 100;                                        // Get variance in mm
   return pos;
}
