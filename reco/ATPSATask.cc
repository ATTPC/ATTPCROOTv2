#include "ATPSATask.hh"
#include "ATPSA.hh"
#include "ATPSASimple.hh"
#include "ATPSASimple2.hh"
#include "ATPSAProto.hh"
#include "ATPSAFilter.hh"
#include "ATPSAFull.hh"
#include "ATPSAProtoFull.hh"


// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

#include <iostream>
#ifdef _OPENMP
#include <omp.h>
#endif

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(ATPSATask);

ATPSATask::ATPSATask()
{
  fLogger = FairLogger::GetLogger();
  fPar = NULL;

  fIsPersistence = kFALSE;
  fIsBGPK = kFALSE;
  fIsPeakFinder = kFALSE;
  fIsMaxFinder = kFALSE;
  fIsBaseCorr = kFALSE;
  fIsTimeCorr = kFALSE;

  fEventHArray = new TClonesArray("ATEvent");
  //fAuxChannels.clear();

  fPSAMode = 2;

  fMeanK = 10;
  fStdDev = 0.01;

  fTBRange.first  = 0;
  fTBRange.second = 512;


}

ATPSATask::~ATPSATask()
{
}

void ATPSATask::SetPSAMode(Int_t value)                     { fPSAMode = value; }
void ATPSATask::SetPersistence(Bool_t value)                { fIsPersistence = value; }
void ATPSATask::SetThreshold(Double_t threshold)            { fThreshold = threshold; }
void ATPSATask::SetBackGroundPeakFinder(Bool_t value)       { fIsBGPK = value;}
void ATPSATask::SetPeakFinder()                             { fIsPeakFinder= kTRUE;fIsMaxFinder= kFALSE;}
void ATPSATask::SetMaxFinder()                              { fIsMaxFinder= kTRUE;fIsPeakFinder= kFALSE;}
void ATPSATask::SetBaseCorrection(Bool_t value)             { fIsBaseCorr = value;}
void ATPSATask::SetTimeCorrection(Bool_t value)             { fIsTimeCorr = value;}
//void ATPSATask::EnableAuxChannels(std::vector<Int_t> AuxCh) { fAuxChannels = AuxCh;}

void ATPSATask::SetMeanK(Int_t value)                       { fMeanK = value;}
void ATPSATask::SetStddevMulThresh(Double_t value)          { fStdDev = value;}
void ATPSATask::SetGainCalibration(TString gainFile)        { fGainFile = gainFile;}
void ATPSATask::SetJitterCalibration(TString jitterFile)    { fJitterFile = jitterFile;}



InitStatus
ATPSATask::Init()
{
  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
    return kERROR;
  }

  fRawEventArray = (TClonesArray *) ioMan -> GetObject("ATRawEvent");
  if (fRawEventArray == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find ATRawEvent array!");
    return kERROR;
  }

  if (fPSAMode == 0) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using ATPSASimple!");

    fPSA = new ATPSASimple();
  } else if (fPSAMode == 1) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using ATPSASimple2!");

    fPSA = new ATPSASimple2();

  } else if (fPSAMode == 2) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using ATPSAProto!");
    fPSA = new ATPSAProto();
    fPSA -> SetTBLimits(fTBRange);

  } else if (fPSAMode == 3) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using ATPSAFilter!");
    fPSA = new ATPSAFilter();
    fPSA -> SetMeanK(fMeanK);
    fPSA -> SetStddevMulThresh(fStdDev);
  
  } else if (fPSAMode == 4) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using ATPSAFull!");

    fPSA = new ATPSAFull();

  } else if (fPSAMode == 5) {
    fLogger -> Info(MESSAGE_ORIGIN, "Using ATPSAProtoFull!");

    fPSA = new ATPSAProtoFull();
    fPSA -> SetTBLimits(fTBRange);

  }  

  fPSA -> SetThreshold((Int_t)fThreshold);
  fPSA -> SetBaseCorrection(fIsBaseCorr);
  fPSA -> SetTimeCorrection(fIsTimeCorr);
  fPSA -> SetGainCalibration(fGainFile);
  fPSA -> SetJitterCalibration(fJitterFile);

   if(fIsBGPK){
	 fLogger -> Info(MESSAGE_ORIGIN, "Suppression of background in Peak Finder Enabled");
         fPSA -> SetBackGroundSuppression();
   }

   if(fIsPeakFinder){
	  fLogger -> Info(MESSAGE_ORIGIN, " Peak Finder enabled for hit pattern reconstruction");
    fPSA -> SetPeakFinder();
  }else if(fIsMaxFinder){
    fLogger -> Info(MESSAGE_ORIGIN, " Maximum Finder enabled for hit pattern reconstruction");
    fPSA -> SetMaxFinder();
  }else if(!fIsMaxFinder && !fIsPeakFinder){
    fLogger -> Error(MESSAGE_ORIGIN, " Please select a method for hit pattern reconstruction");
    return kERROR;
  }

  //Setting Auxiliary channels for prototype

  //if(ATPSA* PSA_ptr = dynamic_cast<ATPSAProto*> (fPSA) ) SetAuxChannel(fAuxChannels);
  ioMan -> Register("ATEventH", "ATTPC", fEventHArray, fIsPersistence);

  return kSUCCESS;
}

void
ATPSATask::SetParContainers()
{
  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");
}

void
ATPSATask::Exec(Option_t *opt)
{


  fEventHArray -> Delete();

  if (fRawEventArray -> GetEntriesFast() == 0)
    return;

  ATRawEvent *rawEvent = (ATRawEvent *) fRawEventArray -> At(0);
  //std::cout << "  Event Number :  " << rawEvent -> GetEventID() << " Valid pads : " << rawEvent -> GetNumPads() << std::endl;

  ATEvent *event = (ATEvent *) new ((*fEventHArray)[0]) ATEvent();

  event->SetEventID(rawEvent->GetEventID());
  event->SetTimestamp(rawEvent->GetTimestamp());

  if (!(rawEvent -> IsGood()))
    event -> SetIsGood(kFALSE);
  else {
    fPSA -> Analyze(rawEvent, event);
    event -> SetIsGood(kTRUE);
  }
}

/*void ATPSATask::SetAuxChannel(std::vector<Int_t> AuxCh)
{

      ATPSA* PSA_ptr;
      if(PSA_ptr = dynamic_cast<ATPSASimple*> (fPSA)) fLogger -> Fatal(MESSAGE_ORIGIN, "SetAuxChannel only implemente for ATPSAProto! Terminating...");
      else if(PSA_ptr = dynamic_cast<ATPSASimple2*> (fPSA) ) fLogger -> Fatal(MESSAGE_ORIGIN, "SetAuxChannel only implemente for ATPSAProto! Terminating...");

      if(AuxCh.size()==0) std::cout<<cRED<<" ATPSATask : ATPSAProto Mode -  No auxiliary channels found --"<<cNORMAL<<std::endl;
      else{
            std::cout<<cGREEN<<" ATPSATask : Auxiliary pads found : "<<std::endl;
            for(Int_t i=0;i<AuxCh.size();i++) std::cout<<"  "<<AuxCh.at(i)<<std::endl;
      }
      std::cout<<cNORMAL<<std::endl;
      //fPSA->SetAuxChannel(AuxCh);

 }*/

void ATPSATask::SetTBLimits(std::pair<Int_t,Int_t> limits)
{
    if(limits.first>=limits.second)
    {
      std::cout<<" Warning ATPSATask::SetTBLimits -  Wrong Time Bucket limits. Setting default limits (0,512) ... "<<"\n";
      

    }else{
      fTBRange.first  = limits.first;
      fTBRange.second = limits.second;
    }



}
