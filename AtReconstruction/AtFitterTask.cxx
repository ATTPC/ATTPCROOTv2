#include "AtFitterTask.h"

#include <FairLogger.h>
#include <FairTask.h>
#include <TClonesArray.h>
#include <fairlogger/Logger.h>
// STL
#include <iostream>
#include <algorithm>

#include "AtPatternEvent.h"
#include "AtTrack.h"
// FAIRROOT classes
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRuntimeDb.h>
// GENFIT2 classes
#include <Track.h>
#include "AtDigiPar.h"
#include "AtFitter.h"
#include "AtGenfit.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

ClassImp(AtFitterTask);

AtFitterTask::AtFitterTask()
{
   fLogger = FairLogger::GetLogger();
   fPar = NULL;
   fIsPersistence = kFALSE;
   fPatternEventArray = new TClonesArray("ATPatternEvent");
   fGenfitTrackArray = new TClonesArray("genfit::Track");
   fFitterAlgorithm = 0;
   fEventCnt = 0;
   fGenfitTrackVector = new std::vector<genfit::Track>();

   fMagneticField = 2.0;
   fMinIterations = 5.0;
   fMaxIterations = 20.0;
   fPDGCode = 2212;
   fMass = 1.00727646;
   fAtomicNumber = 1;
   fNumFitPoints = 0.90;
   fMaxBrho = 3.0;  // Tm
   fMinBrho = 0.01; // Tm

   fELossFile = "";
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

   fPatternEventArray = (TClonesArray *)ioMan->GetObject("AtPatternEvent");
   if (fPatternEventArray == 0) {
      LOG(error) << "Cannot find AtPatternEvent array!";
      return kERROR;
   }

   // Algorithm selection

   if (fFitterAlgorithm == 0) {
      LOG(info) << "Using GENFIT2";
      std::cout << cGREEN << " AtFitterTask::Init - Fit parameters. "
                << "\n";
      std::cout << " Magnetic Field       : " << fMagneticField << " T\n";
      std::cout << " PDG Code             : " << fPDGCode << "\n";
      std::cout << " Mass                 : " << fMass << " amu\n";
      std::cout << " Atomic Number        : " << fAtomicNumber << "\n";
      std::cout << " Number of fit points : " << fNumFitPoints << "\n";
      std::cout << " Maximum iterations   : " << fMaxIterations << "\n";
      std::cout << " Minimum iterations   : " << fMinIterations << "\n";
      std::cout << " Maximum brho         : " << fMaxBrho << "\n";
      std::cout << " Minimum brho         : " << fMinBrho << "\n";
      std::cout << " Energy loss file     : " << fELossFile << "\n";
      std::cout << " --------------------------------------------- " << cNORMAL << "\n";

      fFitter = new AtFITTER::AtGenfit(fMagneticField, fMinBrho, fMaxBrho, fELossFile, fMinIterations, fMaxIterations);
      dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetPDGCode(fPDGCode);
      dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetMass(fMass);
      dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetAtomicNumber(fAtomicNumber);
      dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetNumFitPoints(fNumFitPoints);

   } else if (fFitterAlgorithm == 1) {
      LOG(error) << "Fitter algorithm not defined!";
      return kERROR;
   } else if (fFitterAlgorithm == 2) {
      LOG(error) << "Fitter algorithm not defined!";
      return kERROR;
   }

   // ioMan -> Register("genfitTrackTCA","ATTPC",fGenfitTrackArray, fIsPersistence);
   ioMan->RegisterAny("ATTPC", fGenfitTrackVector, fIsPersistence);

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

void AtFitterTask::Exec(Option_t *option)
{
   if (fPatternEventArray->GetEntriesFast() == 0)
      return;

   fGenfitTrackArray->Clear("C");
   fGenfitTrackVector->clear();

   fFitter->Init();

   std::cout << " Event Counter " << fEventCnt << "\n";

   AtPatternEvent &patternEvent = *((AtPatternEvent *)fPatternEventArray->At(0));
   std::vector<AtTrack> &patternTrackCand = patternEvent.GetTrackCand();
   std::cout << " AtFitterTask:Exec -  Number of candidate tracks : " << patternTrackCand.size() << "\n";

   if (patternTrackCand.size() < 10) {

      for (auto track : patternTrackCand) {

         if (track.GetIsNoise())
            continue;

         genfit::Track *fitTrack = fFitter->FitTracks(&track);
         if (fitTrack != nullptr)
            fGenfitTrackVector->push_back(*static_cast<genfit::Track *>(fitTrack));
      }
   }

   /*if(patternTrackCand.size()>1){
     genfit::Track *fitTrack=fFitter->FitTracks(&patternTrackCand.at(1));
   if(fitTrack!=nullptr)fGenfitTrackVector->push_back(*static_cast<genfit::Track *>(fitTrack));
   }*/

   // if(fitTrack==nullptr)
   // std::cout<<" Nullptr "<<"\n";

   //}

   // }

   // TODO: Genfit block, add a dynamic cast and a try-catch

   /*try {
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

      //auto genfitTracks_ = fGenfitTrackArray->GetEntriesFast();
      //for(auto iTrack=0;iTrack<genfitTracks_;++iTrack){

        //      auto trackTest = *static_cast<genfit::Track*>(fGenfitTrackArray->At(iTrack));
         //     trackTest.Print();
         //     genfit::MeasuredStateOnPlane fitState = trackTest.getFittedState();
          //    fitState.Print();
      //}

   } catch (std::exception &e) {
      std::cout << " " << e.what() << "\n";
      }*/

   ++fEventCnt;
}
