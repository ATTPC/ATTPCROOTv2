#include "ATPRATask.hh"

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

ATPRATask::ATPRATask()
    : FairTask("ATPRATask")
{
    fLogger = FairLogger::GetLogger();
    fLogger->Debug(MESSAGE_ORIGIN, "Defaul Constructor of ATPRATask");
    fPar = NULL;
    fPRAlgorithm = 0;
    kIsPersistence = kFALSE;
    fMinNumHits = 10;
}

ATPRATask::~ATPRATask()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Destructor of ATPRATask");
}

void ATPRATask::SetPersistence(Bool_t value)             { kIsPersistence   = value; }
void ATPRATask::SetPRAlgorithm(Int_t value)              { fPRAlgorithm     = value; }

void ATPRATask::SetParContainers()
{
    fLogger->Debug(MESSAGE_ORIGIN, "SetParContainers of ATPRATask");

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

InitStatus ATPRATask::Init()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Initilization of ATPRATask");

    fPatternEventArray = new TClonesArray("ATPatternEvent");

    if (fPRAlgorithm == 0) {
      fLogger -> Info(MESSAGE_ORIGIN, "Using Track Finder Hierarchical Clustering algorithm");

      fPRA = new ATPATTERN::ATTrackFinderHC();

    } else if (fPRAlgorithm == 1) {
      fLogger -> Info(MESSAGE_ORIGIN, "Using RANSAC algorithm");

      //fPSA = new ATPSASimple2();

    } else if (fPRAlgorithm == 2) {
      fLogger -> Info(MESSAGE_ORIGIN, "Using Hough transform algorithm");
      //fPSA = new ATPSAProto();

    }

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

     ioMan -> Register("ATPatternEvent", "ATTPC",fPatternEventArray, kIsPersistence);

    return kSUCCESS;
}

void ATPRATask::Exec(Option_t* option)
{
    fLogger->Debug(MESSAGE_ORIGIN, "Exec of ATPRATask");

      fPatternEventArray -> Delete();

      if (fEventHArray -> GetEntriesFast() == 0)
       return;

      std::vector<ATHit> hitArray;
      ATEvent &event = *((ATEvent*) fEventHArray->At(0));
 			hitArray = *event.GetHitArray();

      std::cout << "  -I- ATTrackFinderHCTask -  Event Number :  " << event.GetEventID()<<"\n";

      try
      {

        ATPatternEvent *patternEvent = (ATPatternEvent *) new ((*fPatternEventArray)[0]) ATPatternEvent();

        if(hitArray.size()>fMinNumHits) fPRA->FindTracks(event,patternEvent);

      }
       catch (std::runtime_error e)
   		{
   			std::cout << "Analyzation failed! Error: " << e.what() << std::endl;
   		}

      //fEvent  = (ATEvent *) fEventHArray -> At(0);
}

void ATPRATask::Finish()
{
    fLogger->Debug(MESSAGE_ORIGIN, "Finish of ATPRATask");
}
