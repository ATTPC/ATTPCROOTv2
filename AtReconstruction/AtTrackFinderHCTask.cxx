#include "AtTrackFinderHCTask.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

// FAIRROOT classes
#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>

AtTrackFinderHCTask::AtTrackFinderHCTask() : FairTask("AtTrackFinderHCTask")
{
   fLogger = FairLogger::GetLogger();
   LOG(debug) << "Defaul Constructor of AtTrackFinderHCTask";
   fPar = NULL;
   kIsPersistence = kFALSE;
}

AtTrackFinderHCTask::~AtTrackFinderHCTask()
{
   LOG(debug) << "Destructor of AtTrackFinderHCTask";
}

void AtTrackFinderHCTask::SetPersistence(Bool_t value)
{
   kIsPersistence = value;
}

void AtTrackFinderHCTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtHierarchicalClusteringTask";

   FairRun *run = FairRun::Instance();
   if (!run)
      LOG(fatal) << "No analysis run!";

   FairRuntimeDb *db = run->GetRuntimeDb();
   if (!db)
      LOG(fatal) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar");
   if (!fPar)
      LOG(fatal) << "AtDigiPar not found!!";
}

InitStatus AtTrackFinderHCTask::Init()
{
   LOG(debug) << "Initilization of AtTrackFinderHCTaskTask";

   fTrackFinderHCArray = new TClonesArray("AtTrackFinderHC");

   // Get a handle from the IO manager
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == 0) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fEventHArray = (TClonesArray *)ioMan->GetObject("AtEventH");
   if (fEventHArray == 0) {
      LOG(error) << "Cannot find AtEvent array!";
      return kERROR;
   }

   ioMan->Register("AtTrackFinderHC", "AtTPC", fTrackFinderHCArray, kIsPersistence);

   return kSUCCESS;
}

void AtTrackFinderHCTask::Exec(Option_t *option)
{
   LOG(debug) << "Exec of AtTrackFinderHCTask";

   fTrackFinderHCArray->Delete();

   if (fEventHArray->GetEntriesFast() == 0)
      return;

   std::vector<AtHit> hitArray;
   AtEvent &event = *((AtEvent *)fEventHArray->At(0));
   hitArray = *event.GetHitArray();

   std::cout << "  -I- AtTrackFinderHCTask -  Event Number :  " << event.GetEventID() << "\n";

   try {
      // AtTrackFinderHC* trackfinder = (AtTrackFinderHC *) new ((*fTrackFinderHCArray)[0]) AtTrackFinderHC();
      // std::unique_ptr<AtTrackFinderHC> trackfinder = std::make_unique<AtTrackFinderHC>();
      // trackfinder->FindTracks(event);

   } catch (std::runtime_error e) {
      std::cout << "Analyzation failed! Error: " << e.what() << std::endl;
   }

   // fEvent  = (AtEvent *) fEventHArray -> At(0);
}

void AtTrackFinderHCTask::Finish()
{
   LOG(debug) << "Finish of AtTrackFinderHCTask";
}
