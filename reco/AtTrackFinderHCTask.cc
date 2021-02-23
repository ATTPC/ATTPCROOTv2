#include "AtTrackFinderHCTask.h"

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

AtTrackFinderHCTask::AtTrackFinderHCTask()
    : FairTask("AtTrackFinderHCTask")
{
    fLogger = FairLogger::GetLogger();
    fLogger->Debug(MESSAGE_ORIGIN, "Defaul Constructor of AtTrackFinderHCTask");
    fPar = NULL;
    kIsPersistence = kFALSE;
}

AtTrackFinderHCTask::~AtTrackFinderHCTask()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Destructor of AtTrackFinderHCTask");
}

void AtTrackFinderHCTask::SetPersistence(Bool_t value)             { kIsPersistence   = value; }

void AtTrackFinderHCTask::SetParContainers()
{
    fLogger->Debug(MESSAGE_ORIGIN, "SetParContainers of AtHierarchicalClusteringTask");

    FairRun *run = FairRun::Instance();
    if (!run)
      fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

    FairRuntimeDb *db = run -> GetRuntimeDb();
    if (!db)
      fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

    fPar = (AtDigiPar *) db -> getContainer("AtDigiPar");
    if (!fPar)
      fLogger -> Fatal(MESSAGE_ORIGIN, "AtDigiPar not found!!");
}

InitStatus AtTrackFinderHCTask::Init()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Initilization of AtTrackFinderHCTaskTask");

    fTrackFinderHCArray = new TClonesArray("AtTrackFinderHC");

    // Get a handle from the IO manager
    FairRootManager* ioMan = FairRootManager::Instance();
    if (ioMan == 0) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
      return kERROR;
    }

    fEventHArray = (TClonesArray *) ioMan -> GetObject("AtEventH");
    if (fEventHArray == 0) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find AtEvent array!");
      return kERROR;
    }

     ioMan -> Register("AtTrackFinderHC", "AtTPC",fTrackFinderHCArray, kIsPersistence);

    return kSUCCESS;
}

void AtTrackFinderHCTask::Exec(Option_t* option)
{
    fLogger->Debug(MESSAGE_ORIGIN, "Exec of AtTrackFinderHCTask");

      fTrackFinderHCArray -> Delete();

      if (fEventHArray -> GetEntriesFast() == 0)
       return;

      std::vector<AtHit> hitArray;
      AtEvent &event = *((AtEvent*) fEventHArray->At(0));
 			hitArray = *event.GetHitArray();

      std::cout << "  -I- AtTrackFinderHCTask -  Event Number :  " << event.GetEventID()<<"\n";

      try
      {
        //AtTrackFinderHC* trackfinder = (AtTrackFinderHC *) new ((*fTrackFinderHCArray)[0]) AtTrackFinderHC();
        //std::unique_ptr<AtTrackFinderHC> trackfinder = std::make_unique<AtTrackFinderHC>();
        //trackfinder->FindTracks(event);

      }
       catch (std::runtime_error e)
   		{
   			std::cout << "Analyzation failed! Error: " << e.what() << std::endl;
   		}

      //fEvent  = (AtEvent *) fEventHArray -> At(0);
}

void AtTrackFinderHCTask::Finish()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Finish of AtTrackFinderHCTask");
}
