#include "AtPRAtask.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <iostream>
#include <memory>

//// FAIRROOT classes
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRuntimeDb.h"
#include "FairLogger.h"

AtPRAtask::AtPRAtask() : FairTask("AtPRAtask")
{
   fLogger = FairLogger::GetLogger();
   fLogger->Debug(MESSAGE_ORIGIN, "Default Constructor of AtPRAtask");
   fPar = NULL;
   fPRAlgorithm = 0;
   kIsPersistence = kFALSE;
   fMinNumHits = 10;
   fMaxNumHits = 5000;

   fHCs = -1.0;
   fHCk = 19;
   fHCn = 3;
   fHCm = 8;
   fHCr = -1.0;
   fHCa = 0.03;
   fHCt = 3.5;
   fHCpadding = 0.0;

   kSetPrunning = kFALSE;
   fKNN = 5;
   fStdDevMulkNN = 0.0;
   fkNNDist = 10.0;
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
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetTcluster(fHCt);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetScluster(fHCs);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetKtriplet(fHCk);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetNtriplet(fHCn);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetMcluster(fHCm);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetRsmooth(fHCr);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetAtriplet(fHCa);
      // dynamic_cast<AtPATTERN::AtTrackFinderHC*>fPRA->SetPadding(fHCpadding);

      std::cout<<" Track Finder HC parameters (see Dalitz et al.) "<<"\n";
      std::cout<<" T Cluster : "<<fHCt<<"\n";
      std::cout<<" S Cluster : "<<fHCs<<"\n";
      std::cout<<" K Triplet : "<<fHCk<<"\n";
      std::cout<<" N Triplet : "<<fHCn<<"\n";
      std::cout<<" M Cluster : "<<fHCm<<"\n";
      std::cout<<" R Smooth  : "<<fHCr<<"\n";
      std::cout<<" A Triplet : "<<fHCa<<"\n";
      

   } else if (fPRAlgorithm == 1) {
      fLogger->Info(MESSAGE_ORIGIN, "Using RANSAC algorithm");

   } else if (fPRAlgorithm == 2) {
      fLogger->Info(MESSAGE_ORIGIN, "Using Hough transform algorithm");
   }

   // Prunning options
   std::cout << " Track prunning : " << kSetPrunning << "\n";
   if (kSetPrunning) {
      fPRA->SetPrunning();
      std::cout << " Number of k-nearest neighbors (kNN) : " << fKNN << "\n";
      fPRA->SetkNN(fKNN);
      std::cout << " Std deviation multiplier : " << fStdDevMulkNN << "\n";
      fPRA->SetStdDevMulkNN(fStdDevMulkNN);
      std::cout << " kNN Distance threshold : " << fkNNDist << "\n";
      fPRA->SetkNNDist(fkNNDist);
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

      if (hitArray.size() > fMinNumHits && hitArray.size() < fMaxNumHits )
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
