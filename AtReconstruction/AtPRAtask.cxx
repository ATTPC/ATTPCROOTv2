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
   LOG(debug) << "Default Constructor of AtPRAtask";
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
}

AtPRAtask::~AtPRAtask()
{
   LOG(debug) << "Destructor of AtPRAtask";
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
   LOG(debug) << "SetParContainers of AtPRAtask";

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

InitStatus AtPRAtask::Init()
{
   LOG(debug) << "Initilization of AtPRAtask";

   fPatternEventArray = new TClonesArray("AtPatternEvent");

   if (fPRAlgorithm == 0) {
      LOG(info) << "Using Track Finder Hierarchical Clustering algorithm";

      fPRA = new AtPATTERN::AtTrackFinderHC();
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetTcluster(fHCt);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetScluster(fHCs);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetKtriplet(19);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetNtriplet(fHCn);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetMcluster(fHCm);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetRsmooth(fHCr);
      dynamic_cast<AtPATTERN::AtTrackFinderHC *>(fPRA)->SetAtriplet(fHCa);
      // dynamic_cast<AtPATTERN::AtTrackFinderHC*>fPRA->SetPadding(fHCpadding);

   } else if (fPRAlgorithm == 1) {
      LOG(info) << "Using RANSAC algorithm";

      // fPSA = new AtPSASimple2();

   } else if (fPRAlgorithm == 2) {
      LOG(info) << "Using Hough transform algorithm";
      // fPSA = new AtPSAProto();
   }

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

   ioMan->Register("AtPatternEvent", "AtTPC", fPatternEventArray, kIsPersistence);

   return kSUCCESS;
}

void AtPRAtask::Exec(Option_t *option)
{
   LOG(debug) << "Exec of AtPRAtask";

   fPatternEventArray->Delete();

   if (fEventHArray->GetEntriesFast() == 0)
      return;

   std::vector<AtHit> hitArray;
   AtEvent &event = *((AtEvent *)fEventHArray->At(0));
   hitArray = *event.GetHitArray();

   std::cout << "  -I- AtTrackFinderHCTask -  Event Number :  " << event.GetEventID() << "\n";

   try {

      AtPatternEvent *patternEvent = (AtPatternEvent *)new ((*fPatternEventArray)[0]) AtPatternEvent();

      if (hitArray.size() > fMinNumHits && hitArray.size() < fMaxNumHits)
         fPRA->FindTracks(event, patternEvent);

   } catch (std::runtime_error e) {
      std::cout << "Analyzation failed! Error: " << e.what() << std::endl;
   }

   // fEvent  = (AtEvent *) fEventHArray -> At(0);
}

void AtPRAtask::Finish()
{
   LOG(debug) << "Finish of AtPRAtask";
}
