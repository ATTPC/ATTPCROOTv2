#include "AtPSADeconv.h"

#include "AtHit.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h" // for AtPadBase
#include "AtPadFFT.h"

#include <FairLogger.h>

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

const AtPadFFT *AtPSADeconv::GetResponseFFT()
{
   return dynamic_cast<AtPadFFT *>(fResponse.GetAugment("fft"));
}
AtPSADeconv::AtPSADeconv() : AtPSA()
{
   initFFTs();
};

AtPSADeconv::AtPSADeconv(const AtPSADeconv &r)
   : fResponse(r.fResponse), fFFT(nullptr), fFFTbackward(nullptr), fFilterOrder(r.fFilterOrder),
     fCutoffFreq(r.fCutoffFreq)
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

void AtPSADeconv::Init()
{
   AtPSA::Init();
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
 * Set the response function and calculate and save its transform.
 */
void AtPSADeconv::SetResponse(AtPad response)
{
   fResponse = std::move(response);
   initFilter();
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

void AtPSADeconv::initFilter()
{
   fFFT->SetPoints(fResponse.GetADC().data());
   fFFT->Transform();
   auto fft = std::make_unique<AtPadFFT>();
   fft->GetDataFromFFT(fFFT.get());
   fResponse.AddAugment("fft", std::move(fft));
   auto responseFreq = dynamic_cast<AtPadFFT *>(fResponse.GetAugment("fft"));

   // fResponse is now filled with just the plain FFT of the response
   // filter should be lowpass[k]/R[k]
   for (int i = 0; i < fResponse.GetADC().size() / 2 + 1; ++i) {

      auto R = TComplex(responseFreq->GetPointRe(i), responseFreq->GetPointIm(i));
      auto filter = getFilterKernel(i) / R;

      LOG(debug) << i << " " << R << " " << getFilterKernel(i) << " " << filter;
      responseFreq->SetPointRe(i, filter.Re());
      responseFreq->SetPointIm(i, filter.Im());
   }
}
// Assumes the passed pad already has its FFT calculated
AtPSADeconv::HitVector AtPSADeconv::AnalyzeFFTpad(AtPad *pad)
{

   auto padFFT = dynamic_cast<AtPadFFT *>(pad->GetAugment("fft"));
   auto recoFFT = dynamic_cast<AtPadFFT *>(pad->AddAugment("Qreco-fft", std::make_unique<AtPadFFT>()));
   auto respFFT = dynamic_cast<AtPadFFT *>(fResponse.GetAugment("fft"));

   if (padFFT == nullptr || respFFT == nullptr)
      throw std::runtime_error("Missing FFT information in response or pad");

   // Fill the inverse FFT with the input charge
   for (int i = 0; i < 512 / 2 + 1; ++i) {
      auto a = TComplex(padFFT->GetPointRe(i), padFFT->GetPointIm(i));
      auto b = TComplex(respFFT->GetPointRe(i), respFFT->GetPointIm(i));
      LOG(debug) << i << " " << a << " " << b << " " << a * b;
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

   pad->AddAugment("Qreco", std::move(charge));

   return chargeToHits(pad);
}

AtPSADeconv::HitVector AtPSADeconv::AnalyzePad(AtPad *pad)
{
   // If this pad already contains FFT information, then just use it as is.
   if (dynamic_cast<AtPadFFT *>(pad->GetAugment("fft")) != nullptr)
      return AnalyzeFFTpad(pad);

   // Add FFT data to this pad
   fFFT->SetPoints(pad->GetADC().data());
   fFFT->Transform();
   pad->AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));

   // Now process the pad with its fourier transform
   return AnalyzeFFTpad(pad);
}

AtPSADeconv::HitVector AtPSADeconv::chargeToHits(AtPad *pad)
{
   HitVector ret;
   auto charge = dynamic_cast<AtPadArray *>(pad->GetAugment("Qreco"));
   for (auto &ZandQ : getZandQ(charge->GetArray())) {
      XYZPoint pos(pad->GetPadCoord().X(), pad->GetPadCoord().Y(), CalculateZGeo(ZandQ[0]));

      auto posVarXY = getXYhitVariance();
      auto posVarZ = getZhitVariance(0, ZandQ[1]);
      LOG(debug) << "Z(tb): " << ZandQ[0] << " +- " << std::sqrt(ZandQ[1]);
      LOG(debug) << "Z(mm): " << pos.Z() << " +- " << std::sqrt(posVarZ);

      auto hit = std::make_unique<AtHit>(pad->GetPadNum(), pos, ZandQ[2]);
      hit->SetPositionVariance({posVarXY.first, posVarXY.second, posVarZ});
      hit->SetChargeVariance(ZandQ[3]);

      ret.push_back(std::move(hit));
   }
   return ret;
}

AtPSADeconv::HitData AtPSADeconv::getZandQ(const AtPad::trace &charge)
{
   std::array<double, 4> hit{};

   // Get the mean time and total charge
   hit[2] = std::accumulate(charge.begin(), charge.end(), 0.0);
   hit[0] = 0;
   for (int i = 0; i < charge.size(); ++i)
      hit[0] += i * charge[i];
   hit[0] /= hit[2];

   // Get the variance of the time
   hit[1] = 0;
   for (int i = 0; i < charge.size(); ++i) {
      hit[1] += charge[i] * (i - hit[0]) * (i - hit[0]);
   }
   hit[1] /= (hit[2] - 1);

   // Get the variance of the charge
   hit[3] = 0;

   return {hit};
}
double AtPSADeconv::getZhitVariance(double zLoc, double zLocVar) const
{
   // zLocVar is in TB^2
   auto time = zLocVar * fTBTime * fTBTime;           // Get variance in ns^2
   time /= 1000 * 1000;                               // Get variance in us^2
   auto pos = time * fDriftVelocity * fDriftVelocity; // Get variance in cm
   pos *= 100;                                        // Get variance in mm
   return pos;
}
