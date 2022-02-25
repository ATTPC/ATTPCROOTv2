#include "AtTriggerTask.h"
#include "AtTrigger.h"

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

AtTriggerTask::AtTriggerTask() : FairTask("AtTriggerTask"), fIsPersistent(kTRUE) {}

AtTriggerTask::~AtTriggerTask()
{
   LOG(debug) << "Destructor of AtTriggerTask";
}

void AtTriggerTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtTriggerTask";

   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = (AtTriggerPar *)rtdb->getContainer("AtTriggerPar");
}

InitStatus AtTriggerTask::Init()
{
   LOG(debug) << "Initilization of AtTriggerTask";

   FairRootManager *ioman = FairRootManager::Instance();

   //********Get AtEventH and AtRawEvent*************
   fAtEventArray = (TClonesArray *)ioman->GetObject("AtEventH");
   if (fAtEventArray == 0) {
      LOG(error) << "Cannot find fAtEventArray array!";
      return kERROR;
   }

   fAtRawEventArray = (TClonesArray *)ioman->GetObject("AtRawEvent");
   if (fAtRawEventArray == 0) {
      LOG(error) << "Cannot find fAtRawEventArray array!";
      return kERROR;
   }

   //**********SetOutputBranches********************************
   fAtRawEventArray_acc = new TClonesArray("AtRawEvent", 1);
   fAtEventArray_acc = new TClonesArray("AtEvent", 1);

   ioman->Register("Accepted_AtRawEvent", "cbmsim", fAtRawEventArray_acc, fIsPersistent);
   ioman->Register("Accepted_AtEventH", "cbmsim", fAtEventArray_acc, fIsPersistent);

   //**********Get and set parameters*************************
   Double_t read, write, MSB, LSB, width, fraction, threshold, window, height;
   fTrigger = new AtTrigger();

   write = fPar->GetWrite_Clock();
   read = fPar->GetRead_Clock();
   MSB = fPar->GetPad_thres_MSB();
   LSB = fPar->GetPad_thres_LSB();
   width = fPar->GetTrigger_signal_width();
   fraction = fPar->GetTrigger_discriminator_fraction();
   threshold = fPar->GetMultiplicity_threshold();
   window = fPar->GetMultiplicity_window();
   height = fPar->GetTrigger_height();

   fTrigger->SetTriggerParameters(write, read, MSB, LSB, width, fraction, threshold, window, height);
   fTrigger->SetAtMap(fMapPath);
   return kSUCCESS;
}

void AtTriggerTask::SetAtMap(TString mapPath)
{
   fMapPath = mapPath;
}

void AtTriggerTask::Exec(Option_t *option)
{
   LOG(debug) << "Exec of AtTriggerTask";

   //***************Reset everything and load next event****************
   // fAtEventArray_acc     ->Delete();
   // fAtRawEventArray_acc  ->Delete();

   fEvent = NULL;
   fRawEvent = NULL;

   fEvent = (AtEvent *)fAtEventArray->At(0);
   fRawEvent = (AtRawEvent *)fAtRawEventArray->At(0);

   //*****************Check if event will be triggered******************
   fIsTrigger = fTrigger->ImplementTrigger(fRawEvent, fEvent);

   //****************Puts event into new branches***********************
   if (fIsTrigger == kTRUE) {

      std::cerr << "Event triggered by DAQ" << std::endl;

      AtEvent *event_acc = (AtEvent *)new ((*fAtEventArray_acc)[0]) AtEvent(*fEvent);
      AtRawEvent *rawEvent_acc = (AtRawEvent *)new ((*fAtRawEventArray_acc)[0]) AtRawEvent(fRawEvent);
   }
}

ClassImp(AtTriggerTask);
