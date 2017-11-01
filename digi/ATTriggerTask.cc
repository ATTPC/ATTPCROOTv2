#include "ATTriggerTask.hh"
#include "ATTrigger.hh"

// Fair class header
#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"


// STL class headers
#include <cmath>
#include <iostream>
#include <iomanip>

#include "TRandom.h"
#include "TMath.h"
#include "TF1.h"


ATTriggerTask::ATTriggerTask():FairTask("ATTriggerTask"),
fIsPersistent(kTRUE)
{
}

ATTriggerTask::~ATTriggerTask()
{
  fLogger->Debug(MESSAGE_ORIGIN,"Destructor of ATTriggerTask");
}

void
ATTriggerTask::SetParContainers()
{
  fLogger->Debug(MESSAGE_ORIGIN,"SetParContainers of ATTriggerTask");

  FairRunAna* ana = FairRunAna::Instance();
  FairRuntimeDb* rtdb = ana->GetRuntimeDb();
  fPar = (ATTriggerPar*) rtdb->getContainer("ATTriggerPar");
}

InitStatus
ATTriggerTask::Init()
{
  fLogger->Debug(MESSAGE_ORIGIN,"Initilization of ATTriggerTask");

  FairRootManager* ioman = FairRootManager::Instance();

  //********Get ATEventH and ATRawEvent*************
  fATEventArray = (TClonesArray*) ioman->GetObject("ATEventH");
  if (fATEventArray == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find fATEventArray array!");
    return kERROR;
  }

  fATRawEventArray = (TClonesArray*) ioman->GetObject("ATRawEvent");
  if (fATRawEventArray == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find fATRawEventArray array!");
    return kERROR;
}

    //**********SetOutputBranches********************************
    fATRawEventArray_acc 	= new TClonesArray("ATRawEvent", 1);
    fATEventArray_acc    	= new TClonesArray("ATEvent", 1);

    ioman -> Register("Accepted_ATRawEvent", "cbmsim", fATRawEventArray_acc, fIsPersistent);
    ioman -> Register("Accepted_ATEventH", "cbmsim", fATEventArray_acc, fIsPersistent);

    //**********Get and set parameters*************************
  Double_t read, write, MSB, LSB, width, fraction, threshold, window, height;
  fTrigger = new ATTrigger();

  write     = fPar->GetWrite_Clock();
  read      = fPar->GetRead_Clock();
  MSB       = fPar->GetPad_thres_MSB();
  LSB       = fPar->GetPad_thres_LSB();
  width     = fPar->GetTrigger_signal_width();
  fraction  = fPar->GetTrigger_discriminator_fraction();
  threshold = fPar->GetMultiplicity_threshold();
  window    = fPar->GetMultiplicity_window();
  height    = fPar->GetTrigger_height();

  fTrigger->SetTriggerParameters(write, read, MSB, LSB, width, fraction, threshold, window, height);
  fTrigger->SetAtMap(fMapPath);
  return kSUCCESS;
}


void ATTriggerTask::SetAtMap(TString mapPath){
  fMapPath = mapPath;
}


void
ATTriggerTask::Exec(Option_t* option)
{
  fLogger->Debug(MESSAGE_ORIGIN,"Exec of ATTriggerTask");

  //***************Reset everything and load next event****************
    //fATEventArray_acc     ->Delete();
    //fATRawEventArray_acc  ->Delete();

    fEvent    = NULL;
    fRawEvent = NULL;
    

    fEvent    = (ATEvent*) fATEventArray->At(0);
    fRawEvent = (ATRawEvent*) fATRawEventArray->At(0);

  
    //*****************Check if event will be triggered******************
    fIsTrigger = fTrigger->ImplementTrigger(fRawEvent, fEvent);
	
    //****************Puts event into new branches***********************
    if(fIsTrigger == kTRUE){
   
      std::cerr<<"Event triggered by DAQ"<<std::endl;
      
      ATEvent *event_acc = (ATEvent *) new ((*fATEventArray_acc)[0]) ATEvent(fEvent);
      ATRawEvent *rawEvent_acc = (ATRawEvent *) new ((*fATRawEventArray_acc)[0]) ATRawEvent(fRawEvent);

    
   }

}

ClassImp(ATTriggerTask);
