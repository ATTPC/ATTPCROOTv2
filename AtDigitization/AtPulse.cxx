#include "AtPulse.h"

#include "AtContainerManip.h"
#include "AtDigiPar.h"
#include "AtElectronicResponse.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtRawEvent.h"
#include "AtSimulatedPoint.h"

#include <Math/Point2D.h>
#include <TAxis.h>
#include <TRandom.h>

using XYPoint = ROOT::Math::XYPoint;

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
         charge->SetArray(kk - 1, nEle);

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

   ApplyNoiseAndGETgain(pad);
}

void AtPulse::ApplyNoiseAndGETgain(AtPad &pad)
{
   for (int i = 0; i < fNumTbs; ++i) {
      pad.SetADC(i, pad.GetADC(i) * fGETGain);
      if (fNoiseSigma != 0)
         pad.SetADC(i, pad.GetADC(i) * gRandom->Gaus(0, fNoiseSigma));
   }
}

void AtPulse::Reset()
{
   for (auto padNum : fPadsWithCharge)
      fPadCharge[padNum]->Reset();
   fPadsWithCharge.clear();
}
void AtPulse::SetParameters(AtDigiPar *fPar)
{

   fGain = fPar->GetGain();
   fGETGain = fPar->GetGETGain();                    // Get the electronics gain in fC
   fGETGain = 1.602e-19 * 4096 / (fGETGain * 1e-15); // Scale to gain and correct for ADC
   fPeakingTime = fPar->GetPeakingTime() / 1000.;    // in us
   fTBTime = fPar->GetTBTime() / 1000.;              // in us

   fTBEntrance = fPar->GetTBEntrance();
   fTBPadPlane = fTBEntrance - fPar->GetZPadPlane() / 10. / fTBTime / fPar->GetDriftVelocity();

   fGainFunc = std::make_unique<TF1>(
      "gain", "pow([1]+1,[1]+1)/ROOT::Math::tgamma([1]+1)*pow((x/[0]),[1])*exp(-([1]+1)*(x/[0]))", 0,
      fGain * 5); // Polya distribution of gain
   fGainFunc->SetParameter(0, fGain);
   fGainFunc->SetParameter(1, 1);

   auto b = fGainFunc->GetParameter(1);
   fAvgGainDeviation = fGain / (b + 1);
   fAvgGainDeviation *=
      TMath::Sqrt(TMath::Gamma(b + 3) / TMath::Gamma(b + 1) -
                  TMath::Gamma(b + 2) * TMath::Gamma(b + 2) / TMath::Gamma(b + 1) / TMath::Gamma(b + 1));

   // Create all of the historgrmas
   fPadCharge.resize(fMap->GetNumPads());
   for (Int_t padS = 0; padS < fMap->GetNumPads(); padS++) {
      auto maxTime = fTBTime * fNumTbs; // maxTime in ns
      fPadCharge[padS] =
         std::make_unique<TH1F>(TString::Format("%d", padS), TString::Format("%d", padS), fNumTbs, 0, maxTime);
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
   if (fMap->IsInhibited(padNum) == AtMap::InhibitType::kTotal)
      return 0;
   if (numElectrons <= 0)
      return 0;

   double lowGain = 1;
   if (fMap->IsInhibited(padNum) == AtMap::InhibitType::kLowGain)
      lowGain = fLowGainFactor;

   if (fUseFastGain && numElectrons > 10)
      return gRandom->Gaus(fGain, fAvgGainDeviation / TMath::Sqrt(numElectrons)) * lowGain;

   double g = 0;
   for (Int_t i = 0; i < numElectrons; i++)
      g += fGainFunc->GetRandom();
   return g / numElectrons * lowGain;
}
