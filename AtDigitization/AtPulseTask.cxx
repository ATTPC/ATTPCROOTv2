#include "AtPulseTask.h"

#include "AtDigiPar.h"
#include "AtElectronicResponse.h"
#include "AtMCPoint.h"
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h"
#include "AtRawEvent.h"
#include "AtSimulatedPoint.h"

#include <FairLogger.h>
#include <FairParSet.h> // for FairParSet
#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>
#include <FairTask.h>

#include <Math/Vector3D.h>
#include <TAxis.h>
#include <TClonesArray.h>
#include <TF1.h>
#include <TH1.h>
#include <TH2Poly.h>
#include <TMath.h>
#include <TObject.h>
#include <TRandom.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <utility>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

using namespace ElectronicResponse;
AtPulseTask::AtPulseTask() : AtPulseTask("AtPulseTask") {}
AtPulseTask::AtPulseTask(const char *name) : FairTask(name), fRawEventArray(TClonesArray("AtRawEvent", 1)) {}

void AtPulseTask::SetParContainers() {}

/**
 * @brief: Set the response funtion of the electronics for a single electron.
 *
 * @param[in] responseFunction Function pointer of the type `double responseFunction(double reducedTime)` where
 * reducedTime is the time since the arrival of the electron divided by the electronics peaking time.
 */
void AtPulseTask::SetResponseFunction(ResponseFunctionType responseFunction)
{
   LOG(warning) << "AtPulseTask::SetResponseFunction is deprecated and must be called after Init()";
   setParameters(false);
   fResponse = AtReducedTimeResponse(fPeakingTime, responseFunction);
}

void AtPulseTask::setParameters(bool verbose)
{
   auto fPar = dynamic_cast<AtDigiPar *>(FairRunAna::Instance()->GetRuntimeDb()->getContainer("AtDigiPar"));
   fGain = fPar->GetGain();
   fGETGain = fPar->GetGETGain();                    // Get the electronics gain in fC
   fGETGain = 1.602e-19 * 4096 / (fGETGain * 1e-15); // Scale to gain and correct for ADC
   fPeakingTime = fPar->GetPeakingTime() / 1000.;
   fTBTime = fPar->GetTBTime() / 1000.; // in us

   fTBEntrance = fPar->GetTBEntrance();
   fTBPadPlane = fTBEntrance - fPar->GetZPadPlane() / 10. / fTBTime / fPar->GetDriftVelocity();

   // GET gain: 120fC, 240fC, 1pC or 10pC in 4096 channels
   //  1.6e-19*4096/120e-15 = 5.4613e-03
   //  1.6e-19*4096/240e-15 = 2.731e-03
   //  1.6e-19*4096/1e-12 = 6.5536e-04
   //  1.6e-19*4096/10e-12 = 6.5536e-05
   if (verbose) {
      std::cout << "  Gain in AtTPC gas: " << fGain << std::endl;
      std::cout << "  GET Gain: " << fGETGain << std::endl;
      std::cout << "  Electronic peaking time: " << fPeakingTime << " us" << std::endl;
      std::cout << "  Number of pads: " << fMap->GetNumPads() << std::endl;
      std::cout << "  Number of TBs: " << fNumTbs << std::endl;
      std::cout << "  Window at TB: " << fTBEntrance << std::endl;
      std::cout << "  Pad plane at TB: " << fTBPadPlane << std::endl;
   }
   gain = std::make_unique<TF1>("gain",
                                "pow([1]+1,[1]+1)/ROOT::Math::tgamma([1]+1)*pow((x/[0]),[1])*exp(-([1]+1)*(x/[0]))", 0,
                                fGain * 5); // Polya distribution of gain
   gain->SetParameter(0, fGain);
   gain->SetParameter(1, 1);

   auto b = gain->GetParameter(1);
   avgGainDeviation = fGain / (b + 1);
   avgGainDeviation *=
      TMath::Sqrt(TMath::Gamma(b + 3) / TMath::Gamma(b + 1) -
                  TMath::Gamma(b + 2) * TMath::Gamma(b + 2) / TMath::Gamma(b + 1) / TMath::Gamma(b + 1));
   if (verbose)
      std::cout << "  Gas gain deviation: " << avgGainDeviation << std::endl;
}

void AtPulseTask::getPadPlaneAndCreatePadHist()
{
   if (fMap == nullptr)
      LOG(fatal) << "The detector map was not set in AtPulseLineTask!";

   fMap->GeneratePadPlane();
   fPadPlane = fMap->GetPadPlane();

   char buff[100];
   eleAccumulated.reserve(fMap->GetNumPads());
   for (Int_t padS = 0; padS < fMap->GetNumPads(); padS++) {
      sprintf(buff, "%d", padS);
      auto maxTime = fTBTime * fNumTbs; // maxTime in ns
      eleAccumulated.push_back(std::make_unique<TH1F>(buff, buff, fNumTbs, 0, maxTime));
   }
}

InitStatus AtPulseTask::Init()
{
   LOG(INFO) << "Initilization of AtPulseTask";
   FairRootManager *ioman = FairRootManager::Instance();

   fSimulatedPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtSimulatedPoint"));
   if (fSimulatedPointArray == nullptr) {
      LOG(INFO) << "ERROR: Cannot find fSimulatedPointArray array!";
      return kERROR;
   }

   ioman->Register("AtRawEvent", "cbmsim", &fRawEventArray, fIsPersistent);

   setParameters();
   if (fResponse == nullptr)
      fResponse = AtNominalResponse(fPeakingTime);

   getPadPlaneAndCreatePadHist();
   fEventID = 0;
   fRawEvent = nullptr;

   // Retrieve kinematics for each simulated point
   fMCPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtTpcPoint"));
   if (fMCPointArray == nullptr) {
      LOG(error) << "Cannot find fMCPointArray array!";
      return kERROR;
   }

   LOG(info) << " AtPulseTask : Initialization of parameters complete!";
   return kSUCCESS;
}

void AtPulseTask::saveMCInfo(int mcPointID, int padNumber, int trackID)
{
   // Count occurrences of simPoints coming from the same mcPoint
   int count = 0;

   // The same MC point ID is saved per pad only once, but duplicates are allowed in other pads
   for (auto it = MCPointsMap.lower_bound(padNumber); it != MCPointsMap.upper_bound(padNumber); ++it) {
      auto mcPointMap = dynamic_cast<AtMCPoint *>(fMCPointArray->At(mcPointID));
      auto trackIDMap = mcPointMap->GetTrackID();
      if (it->second == mcPointID || (trackID == trackIDMap)) {
         ++count;
         break;
      }
   }

   // insert if the mcPointID and trackID do not both match any existing point
   if (count == 0)
      MCPointsMap.insert(std::make_pair(padNumber, mcPointID));
}

void AtPulseTask::reset()
{
   for (auto &pad : eleAccumulated)
      pad->Reset();

   electronsMap.clear();
   MCPointsMap.clear();
   fRawEventArray.Delete();
   fRawEvent = nullptr;
   fRawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray.ConstructedAt(0));
   fPadPlane->Reset(nullptr);
}

bool AtPulseTask::gatherElectronsFromSimulatedPoint(AtSimulatedPoint *point)
{
   if (point == nullptr)
      return false;

   auto coord = point->GetPosition();
   auto xElectron = coord.x();       // mm
   auto yElectron = coord.y();       // mm
   auto eTime = coord.z();           // us
   eTime += fTBPadPlane * fTBTime;   // correct time for pad plane location
   auto charge = point->GetCharge(); // number of electrons

   auto binNumber = fPadPlane->Fill(xElectron, yElectron);
   auto padNumber = fMap->BinToPad(binNumber);
   auto mcPoint = dynamic_cast<AtMCPoint *>(fMCPointArray->At(point->GetMCPointID()));
   auto trackID = mcPoint->GetTrackID();

   if (padNumber < 0 || padNumber >= fMap->GetNumPads()) {
      LOG(debug) << "Skipping electron...";
      return false;
   }

   if (fIsSaveMCInfo)
      saveMCInfo(point->GetMCPointID(), padNumber, trackID);

   auto totalyInhibited = fMap->IsInhibited(padNumber) == AtMap::InhibitType::kTotal;
   if (!totalyInhibited) {
      auto gAvg = getAvgGETgain(charge);
      eleAccumulated[padNumber]->Fill(eTime, charge * gAvg);
      electronsMap[padNumber] = eleAccumulated[padNumber].get();
   }

   return true;
}

void AtPulseTask::Exec(Option_t *option)
{
   LOG(debug) << "Exec of AtPulseTask";
   reset();

   Int_t nMCPoints = fSimulatedPointArray->GetEntries();
   std::cout << " AtPulseTask: Number of Points " << nMCPoints << std::endl;

   // Distributing electron pulses among the pads
   Int_t skippedPoints = 0;
   for (Int_t i = 0; i < nMCPoints; i++) {
      auto dElectron = dynamic_cast<AtSimulatedPoint *>(fSimulatedPointArray->At(i));
      if (dElectron == nullptr)
         LOG(fatal) << "The TClonesArray AtSimulatedPoint did not contain type AtSimulatedPoint!";
      if (!gatherElectronsFromSimulatedPoint(dElectron))
         skippedPoints++;
   }

   std::cout << "...End of collection of electrons in this event." << std::endl;
   std::cout << "Skipped " << (double)skippedPoints / nMCPoints * 100. << "% of " << nMCPoints << std::endl;

   generateTracesFromGatheredElectrons();
}
double AtPulseTask::nominalResponseFunction(double reducedTime)
{
   double responce = pow(2.718, -3 * reducedTime) * sin(reducedTime) * pow(reducedTime, 3);
   return responce;
}

void AtPulseTask::generateTracesFromGatheredElectrons()
{
   TAxis *axis = eleAccumulated[0]->GetXaxis();
   Double_t binWidth = axis->GetBinWidth(10);

   std::vector<Int_t> signal(fNumTbs, 0);
   for (auto &ite2 : electronsMap) {
      for (Int_t kk = 0; kk < fNumTbs; kk++)
         signal[kk] = 0;
      Int_t thePadNumber = (ite2.first);

      for (Int_t kk = 0; kk < fNumTbs; kk++) {
         if (eleAccumulated[thePadNumber]->GetBinContent(kk + 1) > 0) {
            for (Int_t nn = kk; nn < fNumTbs; nn++) {
               Double_t binCenter = axis->GetBinCenter(kk + 1);
               double time = (((Double_t)nn + 0.5) * binWidth) - binCenter;
               signal[nn] += eleAccumulated[thePadNumber]->GetBinContent(kk + 1) * fResponse(thePadNumber, time);
            }
         }
      }

      // Create pad
      AtPad *pad = fRawEvent->AddPad(thePadNumber);

      auto PadCenterCoord = fMap->CalcPadCenter(thePadNumber);
      pad->SetValidPad(kTRUE);
      pad->SetPadCoord(PadCenterCoord);
      pad->SetPedestalSubtracted(kTRUE);

      auto lowGain = fMap->IsInhibited(thePadNumber) == AtMap::InhibitType::kLowGain ? fLowGainFactor : 1;

      auto charge = std::make_unique<AtPadArray>();

      for (Int_t bin = 0; bin < fNumTbs; bin++) {
         auto adc = signal[bin] * fGETGain * lowGain;
         if (fNoiseSigma != 0)
            adc += gRandom->Gaus(0, fNoiseSigma);
         pad->SetADC(bin, adc); // NOLINT
         if (fUseChargeSave)
            charge->SetArray(bin, eleAccumulated[thePadNumber]->GetBinContent(bin + 1));
      }

      if (fUseChargeSave)
         pad->AddAugment("Q", std::move(charge));
   }

   // if electronsMap.size==0, fEventID still needs to be set
   fRawEvent->SetEventID(fEventID);
   fRawEvent->SetSimMCPointMap(MCPointsMap);
   fRawEvent->SetIsGood(true);

   std::cout << "AtPulseTask Event ID : " << fEventID << "\n";
   ++fEventID;
}

double AtPulseTask::getAvgGETgain(Int_t numElectrons)
{
   if (numElectrons <= 0)
      return 0;

   Double_t gAvg = 0;
   if (fUseFastGain && numElectrons > 10)
      gAvg = gRandom->Gaus(fGain, avgGainDeviation / TMath::Sqrt(numElectrons));
   else {
      for (Int_t i = 0; i < numElectrons; i++)
         gAvg += gain->GetRandom();
      gAvg = gAvg / numElectrons;
   }
   return gAvg;
}

ClassImp(AtPulseTask);
