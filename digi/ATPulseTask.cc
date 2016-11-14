#include "ATPulseTask.hh"
#include "ATHit.hh"

// Fair class header
#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#include "ATVertexPropagator.h"
#include "ATPad.hh"
#include "ATSimulatedPoint.hh"

// STL class headers
#include <cmath>
#include <iostream>
#include <iomanip>

#include "TRandom.h"
#include "TMath.h"
#include "TF1.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"


ATPulseTask::ATPulseTask():FairTask("ATPulseTask"),
fEventID(0)
{

}

ATPulseTask::~ATPulseTask()
{
  fLogger->Debug(MESSAGE_ORIGIN,"Destructor of ATPulseTask");
}

void
ATPulseTask::SetParContainers()
{
  fLogger->Debug(MESSAGE_ORIGIN,"SetParContainers of ATAvalancheTask");

  FairRunAna* ana = FairRunAna::Instance();
  FairRuntimeDb* rtdb = ana->GetRuntimeDb();
  fPar = (ATDigiPar*) rtdb->getContainer("ATDigiPar");
}

InitStatus
ATPulseTask::Init()
{
  fLogger->Debug(MESSAGE_ORIGIN,"Initilization of ATPulseTask");

  FairRootManager* ioman = FairRootManager::Instance();

fDriftedElectronArray = (TClonesArray *) ioman -> GetObject("ATSimulatedPoint");
  if (fDriftedElectronArray == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find fDriftedElectronArray array!");
    return kERROR;
  }
  fRawEventArray  = new TClonesArray("ATRawEvent", 100);        //!< Raw Event array(only one)
  ioman -> Register("ATRawEvent", "cbmsim", fRawEventArray, fIsPersistent);

  fGain = fPar->GetGain();
  std::cout<<"Gain: "<<fGain<<std::endl;

  return kSUCCESS;
}

struct vPad{
  Double_t RawADC[512];
  Int_t padnumb;
};

void
ATPulseTask::Exec(Option_t* option)
{
  fLogger->Debug(MESSAGE_ORIGIN,"Exec of ATPulseTask");


  Int_t nMCPoints = fDriftedElectronArray->GetEntries();
  std::cout<<" ATPulseTask: Number of Points "<<nMCPoints<<std::endl;
  if(nMCPoints<10){
    fLogger->Warning(MESSAGE_ORIGIN, "Not enough hits for digitization! (<10)");
    return;
  }

  Int_t size = fRawEventArray -> GetEntriesFast();
  ATRawEvent* RawEvent = new ((*fRawEventArray)[size]) ATRawEvent();
   Double_t e                       = 2.718;
   Double_t tau                     = 1; //shaping time (us)
   Double_t samplingtime            = 60;
   Double_t samplingrate            = 0.080; //us
   Double_t timeBucket[512]         = {0};
   Double_t digital[512]            = {0};
   Int_t counter                    = 0;
   Double_t output                  = 0;
   std::vector<vPad> padarray;
   Double_t pointmem[10000][3]      = {0};
   Double_t output_sum              = 0;
   Int_t    tbcounter               = 0;
   Double_t c                       = 10;
   Int_t  vsize;
   Int_t cc                         = 0;
   Double_t pBin, g, xElectron, yElectron, eTime, clusterNum, eventID;
   Int_t padNumber;
   TVector3 coord;
   ATSimulatedPoint* dElectron;
   std::vector<Float_t> PadCenterCoord;
   TF1 *gain                        =  new TF1("gain", "4*(x/[0])*pow(2.718, -2*(x/[0]))", 80, 120);//Polya distribution of gain
   gain->SetParameter(0, fGain);


 // ***************Create ATTPC Pad Plane***************************
 TString scriptfile = "Lookup20150611.xml";
 TString dir = getenv("VMCWORKDIR");
 TString scriptdir = dir + "/scripts/"+ scriptfile;

     AtTpcMap *map = new AtTpcMap();
     map->GenerateATTPC();
     Bool_t MapIn = map->ParseXMLMap(scriptdir);
     fPadPlane = map->GetATTPCPlane();

 // ***************Create Time Buckets*******************************
   for(Double_t d = 20; d<61; d+=samplingrate){
     timeBucket[counter] = d;
     counter++;
   }
 // ***************Create Pulse for Each Electron********************
         //std::cout<<"Total number of entries: "<<cGREEN<<nEvents<<cNORMAL<<std::endl;
       for(Int_t iEvents = 1; iEvents<nMCPoints; iEvents++){//for every electron
         dElectron                     = (ATSimulatedPoint*) fDriftedElectronArray -> At(iEvents);
         coord                         = dElectron->GetPosition();
         xElectron                     = coord (0); //mm
         yElectron                     = coord (1); //mm
         eTime                         = coord (2); //us
         counter                       = 0;
         samplingrate                  = 0.08; //us
         pBin                          = fPadPlane->Fill(xElectron,yElectron,c);
         padNumber                     = pBin-1;
         pointmem[1000][3]             =  {0};
         TString check = kTRUE;
         vsize  = padarray.size();
         for(Int_t r = 0; r<vsize; r++){
           if(padNumber == padarray[r].padnumb) check = kFALSE;
         }

         if(check == kTRUE){
           padarray.push_back(vPad());
           padarray[vsize].padnumb = padNumber;
         }

         if(iEvents % 1000 == 0)   std::cout<<"Number of Electrons Processed: "<<cRED<<iEvents<<cNORMAL<<std::endl;

         // *********Pulse Generation for each electron************
         for(Double_t j = eTime; j<eTime+10; j+=samplingrate/5){
           g                     = gain->GetRandom();
           output                = pow(2.718,-3*((j-eTime)/tau))*sin((j-eTime)/tau)*pow((j-eTime)/tau,3)*g;
           pointmem[counter][0]  = j;
           pointmem[counter][1]  = output;
           counter++;

           // **************Once a point is assinged a height in time, it assigns time to a time bucket********************
           for(Int_t k = 0; k<512; k++){//go through all time buckets
             if(j>=timeBucket[k] && j<timeBucket[k+1]){//if point on pulse is in this time bucket, assign it that time bucket
               pointmem[counter][2]  = k;
               break;
             }//end if for time buckets
           }//end assigning pulse to time buckets
         }//end plotting pulse function

         //*********Once pulse is generated, it adds points to the running average********
         Int_t pTimebucket  = pointmem[0][2];
         Int_t A            = 0;
         Int_t nOPoints     = 0;
         Double_t acum[512] = {0};
         while (A<1000){
           if(pTimebucket == pointmem[A][2]){
             acum[pTimebucket]+= pointmem[A][1];
             nOPoints++;
             A++;
           }
           else{
             digital[pTimebucket] = acum[pTimebucket]/nOPoints++;
             pTimebucket          = pointmem[A][2];
             nOPoints             = 0;
           }
         }

         // ********Adds pulse to output array**************
         vsize  = padarray.size();
         for(Int_t y = 0; y<vsize; y++){
           if(padarray[y].padnumb == padNumber){
             for(Int_t del = 0; del<512; del++){//go through every time bucket
               if(digital[del] != 0)  padarray[y].RawADC[del] += digital[del];
             }
             break;
           }
         }
         digital[512]       = {0};
         pointmem[1000][2]  = {0};
       }// end through all electrons

// ***************Set Pad and add to event**************
       vsize = padarray.size();
       Int_t thepad;
       for(Int_t q = 0; q<vsize; q++){
         ATPad *pad = new ATPad();
         thepad = padarray[q].padnumb;
         if(thepad<10240 && thepad>0){
           pad->SetPad(thepad);
           PadCenterCoord = map->CalcPadCenter(thepad);
           pad->SetValidPad(kTRUE);
           pad->SetPadXCoord(PadCenterCoord[0]);
           pad->SetPadYCoord(PadCenterCoord[1]);
           pad->SetPedestalSubtracted(kTRUE);
           for(Int_t p = 0; p<512; p++){
             pad->SetADC(p, padarray[q].RawADC[p]);
           }
           RawEvent->SetPad(pad);
           RawEvent->SetEventID(0);
         }
       }

  return;
}

ClassImp(ATPulseTask);
