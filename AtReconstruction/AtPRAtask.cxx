#include "AtPRAtask.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <iostream>
#include <memory>

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"
#include "FairLogger.h"

AtPRAtask::AtPRAtask() : FairTask("AtPRAtask")
{
   fLogger = FairLogger::GetLogger();
   fLogger->Debug(MESSAGE_ORIGIN, "Defaul Constructor of AtPRAtask");
   fPar = NULL;
   fPRAlgorithm = 0;
   kIsPersistence = kFALSE;
   fMinNumHits = 10;
}

AtPRAtask::~AtPRAtask()
{
   fLogger->Debug(MESSAGE_ORIGIN, "Destructor of AtPRAtask");
}

void AtPRAtask::SetPersistence(Bool_t value)
{
   kIsPersistence = value;
}
void AtPRAtask::SetPRAlgorithm(Int_t value)
{
   fPRAlgorithm = value;
}

void AtPRAtask::SetParContainers()
{
   fLogger->Debug(MESSAGE_ORIGIN, "SetParContainers of AtPRAtask");

   FairRun *run = FairRun::Instance();
   if (!run)
      fLogger->Fatal(MESSAGE_ORIGIN, "No analysis run!");

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      fLogger->Fatal(MESSAGE_ORIGIN, "No runtime database!");

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      fLogger->Fatal(MESSAGE_ORIGIN, "AtDigiPar not found!!");
}

InitStatus AtPRAtask::Init()
{
   fLogger->Debug(MESSAGE_ORIGIN, "Initilization of AtPRAtask");

   fPatternEventArray = new TClonesArray("AtPatternEvent");

   if (fPRAlgorithm == 0) {
      fLogger->Info(MESSAGE_ORIGIN, "Using Track Finder Hierarchical Clustering algorithm");

      fPRA = new AtPATTERN::AtTrackFinderHC();

   } else if (fPRAlgorithm == 1) {
      fLogger->Info(MESSAGE_ORIGIN, "Using RANSAC algorithm");

      // fPSA = new AtPSASimple2();

   } else if (fPRAlgorithm == 2) {
      fLogger->Info(MESSAGE_ORIGIN, "Using Hough transform algorithm");
      // fPSA = new AtPSAProto();
   }

   // Get a handle from the IO manager
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
      return kERROR;
   }

   fEventHArray = (TClonesArray *)ioMan->GetObject("AtEventH");
   if (fEventHArray == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find AtEvent array!");
      return kERROR;
   }

   ioMan->Register("AtPatternEvent", "AtTPC", fPatternEventArray, kIsPersistence);

   return kSUCCESS;
}

void AtPRAtask::Exec(Option_t *option)
{
   fLogger->Debug(MESSAGE_ORIGIN, "Exec of AtPRAtask");

   fPatternEventArray->Delete();

   if (fEventHArray->GetEntriesFast() == 0)
      return;

   std::vector<AtHit> hitArray;
   AtEvent &event = *((AtEvent *)fEventHArray->At(0));
   hitArray = *event.GetHitArray();

   std::cout << "  -I- AtTrackFinderHCTask -  Event Number :  " << event.GetEventID() << "\n";

   try {

      AtPatternEvent *patternEvent = (AtPatternEvent *)new ((*fPatternEventArray)[0]) AtPatternEvent();

      if (hitArray.size() > fMinNumHits)
         fPRA->FindTracks(event, patternEvent);

   } catch (std::runtime_error e) {
      std::cout << "Analyzation failed! Error: " << e.what() << std::endl;
   }

   // fEvent  = (AtEvent *) fEventHArray -> At(0);
}

void AtPRAtask::Finish()
{
   fLogger->Debug(MESSAGE_ORIGIN, "Finish of AtPRAtask");
}
