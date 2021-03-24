#include "ATFitterTask.hh"

// ATTPCROOT classes
#include "ATEvent.hh"
#include "ATTrack.hh"

// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"

// GENFIT2 classes
#include "Track.h"
#include "TrackCand.h"
#include "RKTrackRep.h"
#include "Exception.h"

// STL
#include <iostream>

// ROOT classes
#include "TMatrixDSym.h"
#include "TMatrixD.h"
#include "TMath.h"
#include "TGeoManager.h"
#include "Math/DistFunc.h"

ClassImp(ATFitterTask);

ATFitterTask::ATFitterTask()
{
  fLogger = FairLogger::GetLogger();
  fPar = NULL;
  fIsPersistence = kFALSE;
  fEventHArray = new TClonesArray("ATEvent");
  fFitterAlgorithm = 0;

}

ATFitterTask::~ATFitterTask()
{

}

void ATFitterTask::SetPersistence(Bool_t value) { fIsPersistence = value; }


InitStatus ATFitterTask::Init()
{
  FairRootManager *ioMan = FairRootManager::Instance();
  if (ioMan == 0) {
    fLogger -> Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
    return kERROR;
  }

  fEventHArray = (TClonesArray *) ioMan -> GetObject("ATEventH");
    if (fEventHArray == 0) {
      fLogger -> Error(MESSAGE_ORIGIN, "Cannot find ATEvent array!");
      return kERROR;
    }

  //Algorithm selection
  
    if (fFitterAlgorithm == 0) {
      fLogger -> Info(MESSAGE_ORIGIN, "Using GENFIT2");

      fFitter = new ATFITTER::ATGenfit();

    } else if (fFitterAlgorithm == 1) {
      fLogger -> Error(MESSAGE_ORIGIN, "Fitter algorithm not defined!");
      return kERROR;
    } else if (fFitterAlgorithm == 2) {
      fLogger -> Error(MESSAGE_ORIGIN, "Fitter algorithm not defined!");
      return kERROR;
    }


  return kSUCCESS;
}

void ATFitterTask::SetParContainers()
{
    fLogger->Debug(MESSAGE_ORIGIN, "SetParContainers of ATFitterTask");

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

void ATFitterTask::Exec(Option_t* option)
{

}























