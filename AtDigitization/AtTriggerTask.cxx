#include "AtTriggerTask.h"

#include "AtTrigger.h"

#include <FairLogger.h>
#include <FairParSet.h>
#include <FairTask.h>

#include <TClonesArray.h>
#include <TObject.h>

#include <iostream>
#include <memory>
// Fair class header
#include "AtEvent.h"
#include "AtRawEvent.h"
#include "AtTriggerPar.h"

#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>

AtTriggerTask::AtTriggerTask()
   : FairTask("AtTriggerTask"), fIsPersistent(kTRUE), fAtRawEventArray_acc("AtRawEvent", 1),
     fAtEventArray_acc("AtEvent", 1)
{
}

AtTriggerTask::~AtTriggerTask()
{
   LOG(debug) << "Destructor of AtTriggerTask";
}

void AtTriggerTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtTriggerTask";

   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = dynamic_cast<AtTriggerPar *>(rtdb->getContainer("AtTriggerPar"));
}

InitStatus AtTriggerTask::Init()
{
   LOG(debug) << "Initilization of AtTriggerTask";

   FairRootManager *ioman = FairRootManager::Instance();

   //********Get AtEventH and AtRawEvent*************
   fAtEventArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtEventH"));
   if (fAtEventArray == nullptr) {
      LOG(error) << "Cannot find fAtEventArray array!";
      return kERROR;
   }

   fAtRawEventArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtRawEvent"));
   if (fAtRawEventArray == nullptr) {
      LOG(error) << "Cannot find fAtRawEventArray array!";
      return kERROR;
   }

   //**********SetOutputBranches********************************
   ioman->Register("Accepted_AtRawEvent", "cbmsim", &fAtRawEventArray_acc, fIsPersistent);
   ioman->Register("Accepted_AtEventH", "cbmsim", &fAtEventArray_acc, fIsPersistent);

   //**********Get and set parameters*************************
   Double_t read, write, MSB, LSB, width, fraction, threshold, window, height;
   fTrigger = std::make_unique<AtTrigger>();

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

   fEvent = nullptr;
   fRawEvent = nullptr;

   fEvent = dynamic_cast<AtEvent *>(fAtEventArray->At(0));
   fRawEvent = dynamic_cast<AtRawEvent *>(fAtRawEventArray->At(0));

   //*****************Check if event will be triggered******************
   fIsTrigger = fTrigger->ImplementTrigger(fRawEvent, fEvent);

   //****************Puts event into new branches***********************
   if (fIsTrigger == kTRUE) {

      std::cerr << "Event triggered by DAQ" << std::endl;

      new (fAtEventArray_acc[0]) AtEvent(*fEvent);
      new (fAtRawEventArray_acc[0]) AtRawEvent(*fRawEvent);
   }
}

ClassImp(AtTriggerTask);
