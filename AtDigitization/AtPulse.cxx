#include "AtPulse.h"
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtContainerManip.h"
#include "AtDigiPar.h"
#include "AtElectronicResponse.h"
#include "AtMap.h" // for AtMap, AtMap::InhibitType, AtMap::...
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h" // for AtPadBase
#include "AtRandom.h"
#include "AtRawEvent.h"
#include "AtSimulatedPoint.h"

#include <FairLogger.h> // for Logger, LOG

#include <Math/Point2D.h>
#include <Math/Point2Dfwd.h> // for XYPoint
#include <Rtypes.h>          // for Int_t
#include <TAxis.h>
#include <TMath.h> // for Gamma, Sqrt
#include <TRandom.h>
#include <TString.h> // for TString

#include <cmath>   // for log, lround, fabs
#include <random>  // for gamma_distribution
#include <utility> // for move

using XYPoint = ROOT::Math::XYPoint;
AtPulse::AtPulse(AtMapPtr map, ResponseFunc response) : fMap(map), fResponse(response) // NOLINT
{
   // Make sure the pad plane is generated so we can just access it for reading info (ie multiple threads will not be
   // trying to create the underlying TH2poly.
   fMap->GeneratePadPlane();
}

AtPulse::AtPulse(const AtPulse &other)
   : fMap(other.fMap), fEventID(other.fEventID), fGain(other.fGain), fLowGainFactor(other.fLowGainFactor),
     fGETGain(other.fGETGain), fPeakingTime(other.fPeakingTime), fTBTime(other.fTBTime), fNumTbs(other.fNumTbs),
     fTBEntrance(other.fTBEntrance), fTBPadPlane(other.fTBPadPlane), fResponse(other.fResponse),
     fUseFastGain(other.fUseFastGain), fNoiseSigma(other.fNoiseSigma), fSaveCharge(other.fSaveCharge),
     fDoConvolution(other.fDoConvolution), fAvgGainDeviation(other.fAvgGainDeviation), fPolyaB(other.fPolyaB)
{

   // For reasons unknown, copying the historgam from other (calling copy constructor) causes a huge performance hit.
   // Recreating from scratch does not.
   fPadCharge.resize(fMap->GetNumPads());
   for (Int_t padS = 0; padS < fMap->GetNumPads(); padS++) {
      auto maxTime = fTBTime * fNumTbs; // maxTime in ns
      fPadCharge[padS] =
         std::make_unique<TH1F>(TString::Format("%d", padS), TString::Format("%d", padS), fNumTbs, 0, maxTime);
      fPadCharge[padS]->SetDirectory(nullptr);
   }

   fPadsWithCharge = other.fPadsWithCharge;
}

AtRawEvent AtPulse::GenerateEvent(std::vector<SimPointPtr> &vec)
{
   auto vecPtr = ContainerManip::GetPointerVector(vec);
   return GenerateEvent(vecPtr);
}

AtRawEvent AtPulse::GenerateEvent(std::vector<AtSimulatedPoint *> &vec)
{
   // Reset the list of historgrams with charge info.
   Reset();

   // Fill the charge histogram and apply the
   int numFilled = 0;
   for (auto &point : vec)
      numFilled += AssignElectronsToPad(point);
   LOG(info) << "Skipped " << (double)(vec.size() - numFilled) / vec.size() * 100 << "% of " << vec.size()
             << " points.";

   AtRawEvent ret;
   for (auto padNum : fPadsWithCharge) {
      AtPad *pad = ret.AddPad(padNum);
      FillPad(*pad, *fPadCharge[padNum]);
   }
   return ret;
}

void AtPulse::FillPad(AtPad &pad, TH1F &hist)
{
   auto axis = hist.GetXaxis();
   double binWidth = axis->GetBinWidth(10);
   auto charge = std::make_unique<AtPadArray>();

   for (int kk = 1; kk <= fNumTbs; ++kk) {
      double nEle = hist.GetBinContent(kk);
      if (nEle > 0) {
         // Scale the saved charge down so its closer to reco
         charge->SetArray(kk - 1, nEle * fGETGain * fResponse(pad.GetPadNum(), fPeakingTime));
         // charge->SetArray(kk - 1, nEle);
         if (!fDoConvolution) {
            pad.SetADC(kk - 1, 0);
            continue;
         }

         // Do the convolution
         for (int nn = kk - 1; nn < fNumTbs; ++nn) {
            double binCenter = axis->GetBinCenter(kk);
            double time = ((double)nn + 0.5) * binWidth - binCenter;
            double newADC = pad.GetADC(nn) + nEle * fResponse(pad.GetPadNum(), time);
            pad.SetADC(nn, newADC);
         }
      }
   }

   pad.SetValidPad(true);
   pad.SetPadCoord(fMap->CalcPadCenter(pad.GetPadNum()));
   pad.SetPedestalSubtracted(true);
   if (fSaveCharge)
      pad.AddAugment("Q", std::move(charge));

   ApplyNoise(pad);
}

void AtPulse::ApplyNoise(AtPad &pad)
{
   for (int i = 0; i < fNumTbs; ++i) {
      pad.SetADC(i, pad.GetADC(i) * fGETGain);
      if (fNoiseSigma != 0)
         pad.SetADC(i, pad.GetADC(i) * AtTools::Random::Get().Gaus(0, fNoiseSigma));
   }
}

void AtPulse::Reset()
{
   for (auto padNum : fPadsWithCharge)
      fPadCharge[padNum]->Reset();
   fPadsWithCharge.clear();
}
void AtPulse::SetParameters(const AtDigiPar *fPar)
{

   fGain = fPar->GetGain();
   fGETGain = fPar->GetGETGain();                    // Get the electronics gain in fC
   fGETGain = 1.602e-19 * 4096 / (fGETGain * 1e-15); // Scale to gain and correct for ADC
   fPeakingTime = fPar->GetPeakingTime() / 1000.;    // in us
   fTBTime = fPar->GetTBTime() / 1000.;              // in us

   fTBEntrance = fPar->GetTBEntrance();
   fTBPadPlane = fTBEntrance - fPar->GetZPadPlane() / 10. / fTBTime / fPar->GetDriftVelocity();

   auto b = fPolyaB;
   fAvgGainDeviation = fGain / (b + 1);
   fAvgGainDeviation *=
      TMath::Sqrt(TMath::Gamma(b + 3) / TMath::Gamma(b + 1) -
                  TMath::Gamma(b + 2) * TMath::Gamma(b + 2) / TMath::Gamma(b + 1) / TMath::Gamma(b + 1));

   LOG(info) << "Gain: " << fGain;
   LOG(info) << "GETGain: " << fGETGain;
   LOG(info) << "Peaking time: " << fPeakingTime;
   LOG(info) << "TB Time: " << fTBTime;
   LOG(info) << "TB entrance: " << fTBEntrance;
   LOG(info) << "TB Pad Plane: " << fTBPadPlane;

   // Create all of the historgrmas
   fPadCharge.resize(fMap->GetNumPads());
   for (Int_t padS = 0; padS < fMap->GetNumPads(); padS++) {
      auto maxTime = fTBTime * fNumTbs; // maxTime in ns
      fPadCharge[padS] =
         std::make_unique<TH1F>(TString::Format("%d", padS), TString::Format("%d", padS), fNumTbs, 0, maxTime);
      fPadCharge[padS]->SetDirectory(nullptr);
   }

   // If there is not a response function create a default one
   if (fResponse == nullptr)
      fResponse = ElectronicResponse::AtNominalResponse(fPeakingTime);
}

/**
 * Assign electons to pad and apply the gain from the umegas (including gain reduction from smartzap)
 * Returns if we were able to add the point to a pad.
 */
bool AtPulse::AssignElectronsToPad(AtSimulatedPoint *point)
{
   if (point == nullptr)
      return false;

   auto coord = point->GetPosition();
   auto eTime = coord.z() + fTBPadPlane * fTBTime; // us
   auto charge = point->GetCharge();               // number of electrons
   auto pos = XYPoint(coord.X(), coord.Y());

   int padNum = fMap->GetPadNum(pos);
   if (padNum < 0 || padNum >= fMap->GetNumPads())
      return false;

   double gain = GetGain(padNum, charge);
   if (gain == 0)
      return false;

   fPadCharge[padNum]->Fill(eTime, gain * charge);
   fPadsWithCharge.insert(padNum);
   return true;
}

double AtPulse::GetGain(int padNum, int numElectrons)
{
   return GetGain(fMap->IsInhibited(padNum), numElectrons);
}

double AtPulse::GetGain(AtMap::InhibitType inhibit, int numElectrons)
{
   if (inhibit == AtMap::InhibitType::kTotal)
      return 0;
   if (numElectrons <= 0)
      return 0;

   double lowGain = 1;
   if (inhibit == AtMap::InhibitType::kLowGain)
      lowGain = fLowGainFactor;

   if (fUseFastGain && numElectrons > 10)
      return AtTools::Random::Get().Gaus(fGain, fAvgGainDeviation / TMath::Sqrt(numElectrons)) * lowGain;

   return MeanPolyaGain(fGain, fPolyaB, numElectrons, AtTools::Random::Get()) * lowGain;
}

/**
 * Draw the mean gain of numElectrons electrons, each with an independent gain from the Polya distribution
 *
 *    P(g) = (b+1)^(b+1) / (G Gamma(b+1)) * (g/G)^b * exp(-(b+1) g/G),
 *
 * where G is the mean gain (meanGain) and b the shape. This is a Gamma distribution with shape b+1 and scale G/(b+1).
 *
 * The sum of n independent Gamma(k, s) deviates is a Gamma(n k, s) deviate. So the mean gain of n electrons is one
 * Gamma(n (b+1), G/(b+1)) deviate divided by n. This is one Gamma deviate instead of n table look-ups.
 *
 * When the shape n (b+1) is an integer m <= 16, the Gamma distribution is an Erlang distribution: the sum of m
 * exponentials of mean s. An exponential deviate is -s log(u) with u uniform in (0, 1], so the sum of m of them is
 * -s log(u_1 u_2 ... u_m), the logarithm of the product of m uniforms. That is cheaper than the general Gamma
 * generator, up to about 16 uniforms (measured), above which std::gamma_distribution is as fast.
 *
 * This shortcut is specific to a Gamma-shaped gain. A different single-electron gain distribution must go back to
 * drawing each electron's gain and averaging.
 */
double AtPulse::MeanPolyaGain(double meanGain, double b, int numElectrons, TRandom &rand)
{
   if (numElectrons <= 0)
      return 0;
   constexpr int maxErlangShape = 16;
   double shape = numElectrons * (b + 1);
   double scale = meanGain / (b + 1);

   int erlangOrder = static_cast<int>(std::lround(shape));
   if (erlangOrder <= maxErlangShape && std::fabs(shape - erlangOrder) < 1e-9) {
      double prod = 1;
      for (int i = 0; i < erlangOrder; ++i)
         prod *= rand.Rndm(); // In (0, 1], and the product of 16 cannot underflow
      return -std::log(prod) * scale / numElectrons;
   }

   AtTools::Random::UniformBitGenerator bitGen(rand);
   std::gamma_distribution<double> sum(shape, scale);
   return sum(bitGen) / numElectrons;
}
