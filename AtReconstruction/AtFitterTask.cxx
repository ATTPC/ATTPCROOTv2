#include "AtFitterTask.h"

// AtTPCROOT classes
#include "AtEvent.h"
#include "AtTrack.h"

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

ClassImp(AtFitterTask);

AtFitterTask::AtFitterTask()
{
   fLogger = FairLogger::GetLogger();
   fPar = NULL;
   fIsPersistence = kFALSE;
   fEventHArray = new TClonesArray("AtEvent");
   fFitterAlgorithm = 0;
}

AtFitterTask::~AtFitterTask() {}

void AtFitterTask::SetPersistence(Bool_t value)
{
   fIsPersistence = value;
}

InitStatus AtFitterTask::Init()
{
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

   // Algorithm selection

   if (fFitterAlgorithm == 0) {
      LOG(info) << "Using GENFIT2";

      fFitter = new AtFITTER::AtGenfit();

   } else if (fFitterAlgorithm == 1) {
      LOG(error) << "Fitter algorithm not defined!";
      return kERROR;
   } else if (fFitterAlgorithm == 2) {
      LOG(error) << "Fitter algorithm not defined!";
      return kERROR;
   }

   return kSUCCESS;
}

void AtFitterTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtFitterTask";

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

void AtFitterTask::Exec(Option_t *option) {}
