#include "ATTrackFinderHCTask.hh"

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

ATTrackFinderHCTask::ATTrackFinderHCTask()
    : FairTask("ATTrackFinderHCTask")
{
    fLogger = FairLogger::GetLogger();
    fLogger->Debug(MESSAGE_ORIGIN, "Defaul Constructor of ATTrackFinderHCTask");
    fPar = NULL;
    kIsPersistence = kFALSE;
}

ATTrackFinderHCTask::~ATTrackFinderHCTask()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Destructor of ATTrackFinderHCTask");
}

void ATTrackFinderHCTask::SetPersistence(Bool_t value)             { kIsPersistence   = value; }

void ATTrackFinderHCTask::SetParContainers()
{
    fLogger->Debug(MESSAGE_ORIGIN, "SetParContainers of ATHierarchicalClusteringTask");

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

InitStatus ATTrackFinderHCTask::Init()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Initilization of ATTrackFinderHCTaskTask");

    fTrackFinderHCArray = new TClonesArray("ATTrackFinderHC");

    // Get a handle from the IO manager
    FairRootManager* ioMan = FairRootManager::Instance();
    if (ioMan == 0) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
      return kERROR;
    }

    fEventHArray = (TClonesArray *) ioMan -> GetObject("ATEventH");
    if (fEventHArray == 0) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find ATEvent array!");
      return kERROR;
    }

     ioMan -> Register("ATTrackFinderHC", "ATTPC",fTrackFinderHCArray, kIsPersistence);

    return kSUCCESS;
}

void ATTrackFinderHCTask::Exec(Option_t* option)
{
    fLogger->Debug(MESSAGE_ORIGIN, "Exec of ATTrackFinderHCTask");

      fTrackFinderHCArray -> Delete();

      if (fEventHArray -> GetEntriesFast() == 0)
       return;

      std::vector<ATHit> hitArray;
      ATEvent &event = *((ATEvent*) fEventHArray->At(0));
 			hitArray = *event.GetHitArray();

      std::cout << "  -I- ATTrackFinderHCTask -  Event Number :  " << event.GetEventID()<<"\n";

      try
      {
        //ATTrackFinderHC* trackfinder = (ATTrackFinderHC *) new ((*fTrackFinderHCArray)[0]) ATTrackFinderHC();
        //std::unique_ptr<ATTrackFinderHC> trackfinder = std::make_unique<ATTrackFinderHC>();
        //trackfinder->FindTracks(event);

      }
       catch (std::runtime_error e)
   		{
   			std::cout << "Analyzation failed! Error: " << e.what() << std::endl;
   		}

      //fEvent  = (ATEvent *) fEventHArray -> At(0);
}

void ATTrackFinderHCTask::Finish()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Finish of ATTrackFinderHCTask");
}
