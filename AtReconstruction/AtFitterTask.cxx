#include "AtFitterTask.h"

// AtTPCROOT classes
#include "AtEvent.h"
#include "AtPatternEvent.h"
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
   fPatternEventArray = new TClonesArray("ATPatternEvent");
   fGenfitTrackArray = new TClonesArray("genfit::Track");
   fFitterAlgorithm = 0;

   fGenfitTrackVector = new std::vector<genfit::Track>();
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
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find RootManager!");
      return kERROR;
   }

   fPatternEventArray = (TClonesArray *)ioMan->GetObject("AtPatternEvent");
   if (fPatternEventArray == 0) {
      fLogger->Error(MESSAGE_ORIGIN, "Cannot find AtPatternEvent array!");
      return kERROR;
   }

   // Algorithm selection

   if (fFitterAlgorithm == 0) {
      fLogger->Info(MESSAGE_ORIGIN, "Using GENFIT2");

      fFitter = new AtFITTER::AtGenfit();

   } else if (fFitterAlgorithm == 1) {
      fLogger->Error(MESSAGE_ORIGIN, "Fitter algorithm not defined!");
      return kERROR;
   } else if (fFitterAlgorithm == 2) {
      fLogger->Error(MESSAGE_ORIGIN, "Fitter algorithm not defined!");
      return kERROR;
   }

   // ioMan -> Register("genfitTrackTCA","ATTPC",fGenfitTrackArray, fIsPersistence);
   ioMan->RegisterAny("ATTPC", fGenfitTrackVector, fIsPersistence);

   return kSUCCESS;
}

void AtFitterTask::SetParContainers()
{
   fLogger->Debug(MESSAGE_ORIGIN, "SetParContainers of AtFitterTask");

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

void AtFitterTask::Exec(Option_t *option)
{
   if (fPatternEventArray->GetEntriesFast() == 0)
      return;

   fGenfitTrackArray->Delete();
   fGenfitTrackVector->clear();

   fFitter->Init();

   AtPatternEvent &patternEvent = *((AtPatternEvent *)fPatternEventArray->At(0));

   fFitter->FitTracks(patternEvent);

   // TODO: Genfit block, add a dynamic cast and a try-catch

   try {
      auto genfitTrackArray = dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->GetGenfitTrackArray();
      auto genfitTracks = genfitTrackArray->GetEntriesFast();

      for (auto iTrack = 0; iTrack < genfitTracks; ++iTrack) {
         new ((*fGenfitTrackArray)[iTrack]) genfit::Track(*static_cast<genfit::Track *>(genfitTrackArray->At(iTrack)));
         // auto trackTest = *static_cast<genfit::Track*>(genfitTrackArray->At(iTrack));
         // trackTest.Print();
         // genfit::MeasuredStateOnPlane fitState = trackTest.getFittedState();
         // fitState.Print();
         fGenfitTrackVector->push_back(*static_cast<genfit::Track *>(genfitTrackArray->At(iTrack)));
      }

      /*auto genfitTracks_ = fGenfitTrackArray->GetEntriesFast();
      for(auto iTrack=0;iTrack<genfitTracks_;++iTrack){

              auto trackTest = *static_cast<genfit::Track*>(fGenfitTrackArray->At(iTrack));
              trackTest.Print();
              genfit::MeasuredStateOnPlane fitState = trackTest.getFittedState();
              fitState.Print();
      } */

   } catch (std::exception &e) {
      std::cout << " " << e.what() << "\n";
   }
}
