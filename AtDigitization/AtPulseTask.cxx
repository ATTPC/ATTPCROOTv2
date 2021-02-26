#include "AtPulseTask.h"
#include "AtHit.h"
#include "AtTpcPoint.h"

// Fair class header
#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#include "AtVertexPropagator.h"
#include "AtPad.h"
#include "AtSimulatedPoint.h"

// STL class headers
#include <cmath>
#include <iostream>
#include <iomanip>

#include "TRandom.h"
#include "TMath.h"
#include "TF1.h"
#include "TH1.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

AtPulseTask::AtPulseTask() : FairTask("AtPulseTask"), fEventID(0)
{
   LOG(INFO) << "Constructor of AtPulseTask" << FairLogger::endl;
   fIsSaveMCInfo = kFALSE;
}

AtPulseTask::~AtPulseTask()
{
   LOG(INFO) << "Destructor of AtPulseTask" << FairLogger::endl;
   for (Int_t padS = 0; padS < 10240; padS++) {
      delete eleAccumulated[padS];
   }
   delete[] eleAccumulated;
}

void AtPulseTask::SetInhibitMaps(TString inimap, TString lowgmap, TString xtalkmap)
{
   fIniMap = inimap;
   fLowgMap = lowgmap;
   fXtalkMap = xtalkmap;
}

void AtPulseTask::SetParContainers()
{
   LOG(INFO) << "SetParContainers of AtPulseTask" << FairLogger::endl;
   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = (AtDigiPar *)rtdb->getContainer("AtDigiPar");
}

InitStatus AtPulseTask::Init()
{
   LOG(INFO) << "Initilization of AtPulseTask" << FairLogger::endl;
   FairRootManager *ioman = FairRootManager::Instance();

   fDriftedElectronArray = (TClonesArray *)ioman->GetObject("AtSimulatedPoint");
   if (fDriftedElectronArray == 0) {
      LOG(INFO) << "ERROR: Cannot find fDriftedElectronArray array!" << FairLogger::endl;
      return kERROR;
   }

   fRawEventArray = new TClonesArray("AtRawEvent", 1); //!< Raw Event array (only one)
   ioman->Register("AtRawEvent", "cbmsim", fRawEventArray, fIsPersistent);

   // Retrieve kinematics for each simulated point
   fMCPointArray = (TClonesArray *)ioman->GetObject("AtTpcPoint");
   if (fMCPointArray == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find fMCPointArray array!");
      return kERROR;
   }

   fGain = fPar->GetGain();
   fGETGain = fPar->GetGETGain();                    // Get the electronics gain in fC
   fGETGain = 1.602e-19 * 4096 / (fGETGain * 1e-15); // Scale to gain and correct for ADC
   fPeakingTime = fPar->GetPeakingTime();

   std::cout << "  Gain in AtTPC gas: " << fGain << std::endl;
   std::cout << "  GET Gain: " << fGETGain << std::endl;
   std::cout << "  Electronic peaking time: " << fPeakingTime << " ns" << std::endl;

   gain = new TF1("gain", "pow([1]+1,[1]+1)/ROOT::Math::tgamma([1]+1)*pow((x/[0]),[1])*exp(-([1]+1)*(x/[0]))", 0,
                  fGain * 5); // Polya distribution of gain
   gain->SetParameter(0, fGain);
   gain->SetParameter(1, 1);

   // TODO: THIS SHOULD BE TAKEN FROM A NEW PARAMETER AS THE GAS GAIN
   // GET gain: 120fC, 240fC, 1pC or 10pC in 4096 channels
   //  1.6e-19*4096/120e-15 = 5.4613e-03
   //  1.6e-19*4096/240e-15 = 2.731e-03
   //  1.6e-19*4096/1e-12 = 6.5536e-04
   //  1.6e-19*4096/10e-12 = 6.5536e-05
   //  fGETGain = 5.4613e-03; //electrones(carga)*4096/120 fC
   // std::cout<<"  GET Gain: " << fGETGain << std::endl;

   fTBTime = fPar->GetTBTime(); // Assuming 80 ns bucket
   fNumTbs = fPar->GetNumTbs();

   // ***************Create AtTPC Pad Plane***************************
   TString scriptfile = "Lookup20150611.xml";
   TString dir = getenv("VMCWORKDIR");
   TString scriptdir = dir + "/scripts/" + scriptfile;

   fMap = new AtTpcMap();
   fMap->GenerateATTPC();
   Bool_t MapIn = fMap->ParseXMLMap(scriptdir);
   fPadPlane = fMap->GetATTPCPlane();
   if (fIsInhibitMap)
      fMap->ParseInhibitMap(fIniMap, fLowgMap, fXtalkMap);

   fEventID = 0;
   fRawEvent = NULL;

   /*TF1 ePulse(Form("ePulse_%i",iEvents),PadResponse,0,100,3);
     ePulse.SetParameter(0,fGain);
     ePulse.SetParameter(1,eTime);
     ePulse.SetParameter(2,tau);*/

   char buff[100];
   eleAccumulated = new TH1F *[10241];
   for (Int_t padS = 0; padS < 10240; padS++) {
      sprintf(buff, "%d", padS);
      eleAccumulated[padS] =
         new TH1F(buff, buff, fNumTbs, 0, fTBTime * fNumTbs / 1000); // max time in microseconds from Time bucket size
   }

   return kSUCCESS;
}

Double_t PadResponse(Double_t *x, Double_t *par)
{
   return par[0] * TMath::Exp(-3.0 * (x[0] - par[1]) / par[2]) * TMath::Sin((x[0] - par[1]) / par[2]) *
          TMath::Power((x[0] - par[1]) / par[2], 3);
}

void AtPulseTask::Exec(Option_t *option)
{

   LOG(INFO) << "Exec of AtPulseTask" << FairLogger::endl;

   Double_t tau = fPeakingTime / 1000.; // shaping time (us)

   for (Int_t padS = 0; padS < 10240; padS++)
      eleAccumulated[padS]->Reset();

   electronsMap.clear();
   MCPointsMap.clear();

   Int_t nMCPoints = fDriftedElectronArray->GetEntries();
   std::cout << " AtPulseTask: Number of Points " << nMCPoints << std::endl;
   if (nMCPoints < 10) {
      LOG(INFO) << "Not enough hits for digitization! (<10)" << FairLogger::endl;
      fRawEvent->SetEventID(fEventID);
      ++fEventID;
      return;
   }

   fRawEventArray->Delete();
   fRawEvent = NULL;
   fRawEvent = (AtRawEvent *)fRawEventArray->ConstructedAt(0);
   fPadPlane->Reset(0);
   Int_t size = fRawEventArray->GetEntriesFast(); // will be always 1

   // Distributing electron pulses among the pads
   for (Int_t iEvents = 0; iEvents < nMCPoints; iEvents++) { // for every electron

      auto dElectron = (AtSimulatedPoint *)fDriftedElectronArray->At(iEvents);
      auto coord = dElectron->GetPosition();
      auto xElectron = coord(0); // mm
      auto yElectron = coord(1); // mm
      auto eTime = coord(2);     // us
      auto padNumber = (int)fPadPlane->Fill(xElectron, yElectron) - 1;
      auto mcPoint = (AtTpcPoint *)fMCPointArray->At(dElectron->GetPointID());
      auto trackID = mcPoint->GetTrackID();

      // std::cout<<" Electron Number "<<iEvents<<" mcPointID : "<<dElectron->GetPointID()<<" Pad number "<<padNumber
      // <<"\n";

      if (padNumber < 0 || padNumber > 10240)
         continue;

      if (fIsSaveMCInfo) {

         // Count occurrences of electrons coming from the same point
         int val = dElectron->GetPointID(), count = 0;

         // Only one (or more depending on count) MC point with the same ID is saved globally, pads can contain several
         // MC points. If a pad contains the same MC point ID, it is discarded.
         /*for (const auto& entry : MCPointsMap){
         if (entry.second == val)
         {
             ++count;
             break;
         }
          }*/

         // The same MC point ID is saved per pad only once, but duplicates are allowed in other pads
         std::multimap<Int_t, std::size_t>::iterator it;
         for (it = MCPointsMap.equal_range(padNumber).first; it != MCPointsMap.equal_range(padNumber).second; ++it) {
            auto mcPointMap = (AtTpcPoint *)fMCPointArray->At(val);
            auto trackIDMap = mcPointMap->GetTrackID();
            if (it->second == val || (trackID == trackIDMap)) {
               // std::cout<<" padNumber "<<padNumber<<" it->second "<<it->second<<" "<<" val "<<val<<" count
               // "<<count<<"\n";
               ++count;
               break;
            }
         }

         if (count == 0) {
            auto const insertionResult = MCPointsMap.insert(std::make_pair(padNumber, dElectron->GetPointID()));
            // std::cout<<" MC points  "<<MCPointsMap.count(padNumber)<<"	in pad "<<padNumber<<" with Point ID
            // "<<dElectron->GetPointID()<<" with track ID "<<mcPoint->GetTrackID()<<" with A/Z
            // "<<mcPoint->GetMassNum()<<"/"<<mcPoint->GetAtomicNum()<<"\n";
         }
      }

      // std::cout<<padNumber<<"  "<<coord(0)<<"  "<<coord(1)<<"  "<<coord(2)<<"\n";
      // std::cout << eTime << " " <<std::endl;
      Bool_t IsInhibited = fMap->GetIsInhibited(padNumber);
      if (!IsInhibited) {
         std::map<Int_t, TH1F *>::iterator ite = electronsMap.find(padNumber);
         if (ite == electronsMap.end()) {
            eleAccumulated[padNumber]->Fill(eTime);
            electronsMap[padNumber] = eleAccumulated[padNumber];
         } else {
            eleAccumulated[padNumber] = (ite->second);
            eleAccumulated[padNumber]->Fill(eTime);
            electronsMap[padNumber] = eleAccumulated[padNumber];
         }
      }
   }
   // std::cout << "...End of collection of electrons in this event."<< std::endl;
   TAxis *axis = eleAccumulated[0]->GetXaxis();
   Double_t binWidth = axis->GetBinWidth(10);

   // output.cd();

   std::vector<Float_t> PadCenterCoord;
   std::map<Int_t, TH1F *>::iterator ite2 = electronsMap.begin();
   Int_t signal[fNumTbs];

   // Double_t *thePar = new Double_t[3];
   while (ite2 != electronsMap.end()) {
      for (Int_t kk = 0; kk < fNumTbs; kk++)
         signal[kk] = 0;
      Int_t thePadNumber = (ite2->first);
      eleAccumulated[thePadNumber] = (ite2->second);
      // Set Pad and add to event
      AtPad *pad = new AtPad();
      // std::cout<<" Pad number "<<thePadNumber<<"\n";

      for (Int_t kk = 0; kk < fNumTbs; kk++) {
         if (eleAccumulated[thePadNumber]->GetBinContent(kk) > 0) {
            for (Int_t nn = kk; nn < fNumTbs; nn++) {
               Double_t binCenter = axis->GetBinCenter(kk);
               Double_t factor = (((((Double_t)nn) + 0.5) * binWidth) - binCenter) / tau;
               Double_t factor_2 = pow(2.718, -3 * factor) * sin(factor) * pow(factor, 3);
               signal[nn] += eleAccumulated[thePadNumber]->GetBinContent(kk) * pow(2.718, -3 * factor) * sin(factor) *
                             pow(factor, 3);
               // if(signal[nn]>0)std::cout<<" Bin "<<kk<<" eleAcc "<<eleAccumulated[thePadNumber]->GetBinContent(kk)<<"
               // Signal "<<signal[nn]<<" factor "<<factor<<" factor 2 "<<factor_2<<"\n";
            }
         }
      }

      pad->SetPad(thePadNumber);
      PadCenterCoord = fMap->CalcPadCenter(thePadNumber);
      pad->SetValidPad(kTRUE);
      pad->SetPadXCoord(PadCenterCoord[0]);
      pad->SetPadYCoord(PadCenterCoord[1]);
      // std::cout<<" X "<<PadCenterCoord[0]<<" Y "<<PadCenterCoord[1]<<"\n";
      pad->SetPedestalSubtracted(kTRUE);
      Double_t gAvg = 0;
      gRandom->SetSeed(0);
      Int_t nEleAcc = eleAccumulated[thePadNumber]->GetEntries();
      for (Int_t i = 0; i < nEleAcc; i++)
         gAvg += gain->GetRandom();
      if (nEleAcc > 0)
         gAvg = gAvg / nEleAcc;

      for (Int_t bin = 0; bin < fNumTbs; bin++) {
         pad->SetADC(bin, signal[bin] * gAvg * fGETGain);
         // if(signal[bin]!=0)std::cout<<" bin "<<bin<<" signal "<<signal[bin]<<" gAvg "<<gAvg<<" GET gain
         // "<<fGETGain<<" nEleAcc "<<nEleAcc<<"\n";
      }

      fRawEvent->SetPad(pad);
      delete pad;
      // std::cout <<" Event "<<aux<<" "<<electronsMap.size()<< "*"<< std::flush;
      ite2++;
   }
   // if electronsMap.size==0, fEventID still needs to be set
   fRawEvent->SetEventID(fEventID);
   fRawEvent->SetSimMCPointMap(MCPointsMap);

   std::cout << "AtPulseTask Event ID : " << fEventID << "\n";
   ++fEventID;
   return;
}

ClassImp(AtPulseTask);
