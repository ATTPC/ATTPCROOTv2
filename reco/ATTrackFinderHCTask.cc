#include "ATTrackFinderHCTask.hh"

#include <chrono>
#include <iostream>
#include <thread>
#include <iostream>

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
