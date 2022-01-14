#include "AtPulseTask.h"

#include "AtDigiPar.h"
#include "AtHit.h"
#include "AtMap.h"
#include "AtPad.h"
#include "AtRawEvent.h"
#include "AtSimulatedPoint.h"
#include "AtTpcPoint.h"
#include "AtVertexPropagator.h"

// Fair class header
#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

// STL class headers
#include <cmath>
#include <iostream>
#include <iomanip>

#include "TClonesArray.h"
#include "TF1.h"
#include "TH1.h"
#include "TH2Poly.h"
#include "TMath.h"
#include "TRandom.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

AtPulseTask::AtPulseTask() : FairTask("AtPulseTask"), fEventID(0)
{
   LOG(debug) << "Constructor of AtPulseTask";
   fIsSaveMCInfo = kFALSE;
   fGain = 0;
   fGETGain = 0;
   fPeakingTime = 0;
}

AtPulseTask::~AtPulseTask()
{
   LOG(debug) << "Destructor of AtPulseTask";
   for (Int_t padS = 0; padS < fMap->GetNumPads(); padS++) {
      delete eleAccumulated[padS];
   }
   delete[] eleAccumulated;
}

void AtPulseTask::SetParContainers()
{
   LOG(INFO) << "SetParContainers of AtPulseTask";
   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = (AtDigiPar *)rtdb->getContainer("AtDigiPar");
}

InitStatus AtPulseTask::Init()
{
   LOG(INFO) << "Initilization of AtPulseTask";
   FairRootManager *ioman = FairRootManager::Instance();

   fDriftedElectronArray = (TClonesArray *)ioman->GetObject("AtSimulatedPoint");
   if (fDriftedElectronArray == 0) {
      LOG(INFO) << "ERROR: Cannot find fDriftedElectronArray array!";
      return kERROR;
   }

   fRawEventArray = new TClonesArray("AtRawEvent", 1); //!< Raw Event array (only one)
   ioman->Register("AtRawEvent", "cbmsim", fRawEventArray, fIsPersistent);

   // Retrieve kinematics for each simulated point
   fMCPointArray = (TClonesArray *)ioman->GetObject("AtTpcPoint");
   if (fMCPointArray == 0) {
      LOG(error) << "Cannot find fMCPointArray array!";
      return kERROR;
   }

   fGain = fPar->GetGain();
   fGETGain = fPar->GetGETGain();                    // Get the electronics gain in fC
   fGETGain = 1.602e-19 * 4096 / (fGETGain * 1e-15); // Scale to gain and correct for ADC
   fPeakingTime = fPar->GetPeakingTime();

   // GET gain: 120fC, 240fC, 1pC or 10pC in 4096 channels
   //  1.6e-19*4096/120e-15 = 5.4613e-03
   //  1.6e-19*4096/240e-15 = 2.731e-03
   //  1.6e-19*4096/1e-12 = 6.5536e-04
   //  1.6e-19*4096/10e-12 = 6.5536e-05
   if (fMap == nullptr)
      LOG(fatal) << "The detector map was not set in AtPulseTask!";

   std::cout << "  Gain in AtTPC gas: " << fGain << std::endl;
   std::cout << "  GET Gain: " << fGETGain << std::endl;
   std::cout << "  Electronic peaking time: " << fPeakingTime << " ns" << std::endl;
   std::cout << "  Number of pads: " << fMap->GetNumPads() << std::endl;

   gain = new TF1("gain", "pow([1]+1,[1]+1)/ROOT::Math::tgamma([1]+1)*pow((x/[0]),[1])*exp(-([1]+1)*(x/[0]))", 0,
                  fGain * 5); // Polya distribution of gain
   gain->SetParameter(0, fGain);
   gain->SetParameter(1, 1);

   fTBTime = fPar->GetTBTime(); // Assuming 80 ns bucket
   fNumTbs = fPar->GetNumTbs();

   // ***************Create AtTPC Pad Plane***************************
   fMap->GenerateAtTpc();
   fPadPlane = fMap->GetAtTpcPlane();
   fEventID = 0;
   fRawEvent = nullptr;

   char buff[100];
   eleAccumulated = new TH1F *[fMap->GetNumPads() + 1];
   for (Int_t padS = 0; padS < fMap->GetNumPads(); padS++) {
      sprintf(buff, "%d", padS);
      eleAccumulated[padS] =
         new TH1F(buff, buff, fNumTbs, 0, fTBTime * fNumTbs / 1000); // max time in microseconds from Time bucket size
   }

   std::cout << " AtDigitalization : Initialization of parameters complete!  "
             << "\n";

   return kSUCCESS;
}

Double_t PadResponse(Double_t *x, Double_t *par)
{
   return par[0] * TMath::Exp(-3.0 * (x[0] - par[1]) / par[2]) * TMath::Sin((x[0] - par[1]) / par[2]) *
          TMath::Power((x[0] - par[1]) / par[2], 3);
}

void AtPulseTask::saveMCInfo(int mcPointID, int padNumber, int trackID)
{
   // Count occurrences of electrons coming from the same point
   int count = 0;

   // The same MC point ID is saved per pad only once, but duplicates are allowed in other pads
   std::multimap<Int_t, std::size_t>::iterator it;
   for (it = MCPointsMap.equal_range(padNumber).first; it != MCPointsMap.equal_range(padNumber).second; ++it) {
      auto mcPointMap = (AtTpcPoint *)fMCPointArray->At(mcPointID);
      auto trackIDMap = mcPointMap->GetTrackID();
      if (it->second == mcPointID || (trackID == trackIDMap)) {
         ++count;
         break;
      }
   }

   // insert if the mcPointID and trackID do not both match any existing point
   if (count == 0)
      auto const insertionResult = MCPointsMap.insert(std::make_pair(padNumber, mcPointID));
}
void AtPulseTask::Exec(Option_t *option)
{

   LOG(INFO) << "Exec of AtPulseTask";

   Double_t tau = fPeakingTime / 1000.; // shaping time (us)

   for (Int_t padS = 0; padS < fMap->GetNumPads(); padS++)
      eleAccumulated[padS]->Reset();

   electronsMap.clear();
   MCPointsMap.clear();

   Int_t nMCPoints = fDriftedElectronArray->GetEntries();
   std::cout << " AtPulseTask: Number of Points " << nMCPoints << std::endl;

   fRawEventArray->Delete();
   fRawEvent = nullptr;
   fRawEvent = (AtRawEvent *)fRawEventArray->ConstructedAt(0);
   fPadPlane->Reset(0);

   // Distributing electron pulses among the pads
   for (Int_t iEvents = 0; iEvents < nMCPoints; iEvents++) { // for every electron

      auto dElectron = (AtSimulatedPoint *)fDriftedElectronArray->At(iEvents);
      auto coord = dElectron->GetPosition();
      auto xElectron = coord.x();           // mm
      auto yElectron = coord.y();           // mm
      auto eTime = coord.z();               // us
      auto charge = dElectron->GetCharge(); // number of electrons

      auto binNumber = fPadPlane->Fill(xElectron, yElectron);
      auto padNumber = fMap->BinToPad(binNumber);
      auto mcPoint = (AtTpcPoint *)fMCPointArray->At(dElectron->GetMCPointID());
      auto trackID = mcPoint->GetTrackID();

      if (padNumber < 0 || padNumber > fMap->GetNumPads())
         continue;

      if (fIsSaveMCInfo)
         saveMCInfo(dElectron->GetMCPointID(), padNumber, trackID);

      Bool_t IsInhibited = fMap->GetIsInhibited(padNumber);
      if (!IsInhibited) {
         eleAccumulated[padNumber]->Fill(eTime, charge);
         electronsMap[padNumber] = eleAccumulated[padNumber];
      }
   } // end loop over # electrons

   std::cout << "...End of collection of electrons in this event." << std::endl;
   TAxis *axis = eleAccumulated[0]->GetXaxis();
   Double_t binWidth = axis->GetBinWidth(10);

   std::vector<Float_t> PadCenterCoord;
   Int_t signal[fNumTbs];
   for (auto ite2 = electronsMap.begin(); ite2 != electronsMap.end(); ++ite2) {
      for (Int_t kk = 0; kk < fNumTbs; kk++)
         signal[kk] = 0;
      Int_t thePadNumber = (ite2->first);
      // eleAccumulated[thePadNumber] = (ite2->second);

      for (Int_t kk = 0; kk < fNumTbs; kk++) {
         if (eleAccumulated[thePadNumber]->GetBinContent(kk) > 0) {
            for (Int_t nn = kk; nn < fNumTbs; nn++) {
               Double_t binCenter = axis->GetBinCenter(kk);
               Double_t factor = (((((Double_t)nn) + 0.5) * binWidth) - binCenter) / tau;
               Double_t factor_2 = pow(2.718, -3 * factor) * sin(factor) * pow(factor, 3);
               signal[nn] += eleAccumulated[thePadNumber]->GetBinContent(kk) * pow(2.718, -3 * factor) * sin(factor) *
                             pow(factor, 3);
            }
         }
      }

      // Create pad
      auto &pad = fRawEvent->AddPad(thePadNumber);

      PadCenterCoord = fMap->CalcPadCenter(thePadNumber);
      pad.SetValidPad(kTRUE);
      pad.SetPadXCoord(PadCenterCoord[0]);
      pad.SetPadYCoord(PadCenterCoord[1]);
      // std::cout<<" X "<<PadCenterCoord[0]<<" Y "<<PadCenterCoord[1]<<"\n";
      pad.SetPedestalSubtracted(kTRUE);
      Double_t gAvg = 0;
      gRandom->SetSeed(0);
      Int_t nEleAcc = eleAccumulated[thePadNumber]->GetEntries();
      for (Int_t i = 0; i < nEleAcc; i++)
         gAvg += gain->GetRandom();
      if (nEleAcc > 0)
         gAvg = gAvg / nEleAcc;

      for (Int_t bin = 0; bin < fNumTbs; bin++) {
         pad.SetADC(bin, signal[bin] * gAvg * fGETGain);
      }
   }
   // if electronsMap.size==0, fEventID still needs to be set
   fRawEvent->SetEventID(fEventID);
   fRawEvent->SetSimMCPointMap(MCPointsMap);

   std::cout << "AtPulseTask Event ID : " << fEventID << "\n";
   ++fEventID;
   return;
}

ClassImp(AtPulseTask);
