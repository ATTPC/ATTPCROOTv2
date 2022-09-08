#include "AtPRAtask.h"

#include "AtDigiPar.h"       // for AtDigiPar
#include "AtEvent.h"         // for AtEvent
#include "AtPRA.h"           // for AtPRA
#include "AtPatternEvent.h"  // for AtPatternEvent
#include "AtTrackFinderTC.h" // for AtTrackFinderHC

#include <FairLogger.h>      // for LOG, FairLogger
#include <FairRootManager.h> // for FairRootManager
#include <FairRun.h>         // for FairRun
#include <FairRuntimeDb.h>   // for FairRuntimeDb

#include <TObject.h> // for TObject

#include <iostream>  // for operator<<, basic_ostream, cout, ostream
#include <memory>    // for unique_ptr<>::element_type, unique_ptr
#include <stdexcept> // for runtime_error
#include <utility>   // for move
#include <vector>    // for allocator, vector

AtPRAtask::AtPRAtask()
   : fInputBranchName("AtEventH"), fOutputBranchName("AtPatternEvent"), FairTask("AtPRAtask"),
     fPatternEventArray("AtPatternEvent", 1)
{

   LOG(debug) << "Default Constructor of AtPRAtask";
   fPar = nullptr;
   fPRAlgorithm = 0;
   kIsPersistence = kFALSE;
   fMinNumHits = 10;
   fMaxNumHits = 5000;

   fHCs = 0.3;
   fHCk = 19;
   fHCn = 2;
   fHCm = 15;
   fHCr = 2.0;
   fHCa = 0.03;
   fHCt = 4.0;
   fHCpadding = 0.0;

   kSetPrunning = kFALSE;
   fKNN = 5;
   fStdDevMulkNN = 0.0;
   fkNNDist = 10.0;
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

   FairRuntimeDb *db = run->GetRuntimeDb(); // NOLINT
   if (!db)
      LOG(fatal) << "No runtime database!";

   fPar = (AtDigiPar *)db->getContainer("AtDigiPar"); // NOLINT
   if (!fPar)
      LOG(fatal) << "AtDigiPar not found!!";
}

InitStatus AtPRAtask::Init()
{
   LOG(debug) << "Initilization of AtPRAtask";

   if (fPRAlgorithm == 0) {
      LOG(info) << "Using Track Finder TriplClust algorithm";

      fPRA = new AtPATTERN::AtTrackFinderTC();
      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetTcluster(fHCt);
      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetScluster(fHCs);
      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetKtriplet(fHCk);
      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetNtriplet(fHCn);
      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetMcluster(fHCm);
      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetRsmooth(fHCr);
      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetAtriplet(fHCa);
      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetPadding(fHCpadding);

      std::cout << " Track Finder TriplClust parameters (see Dalitz et al.) "
                << "\n";
      std::cout << " T Cluster : " << fHCt << "\n";
      std::cout << " S Cluster : " << fHCs << "\n";
      std::cout << " K Triplet : " << fHCk << "\n";
      std::cout << " N Triplet : " << fHCn << "\n";
      std::cout << " M Cluster : " << fHCm << "\n";
      std::cout << " R Smooth  : " << fHCr << "\n";
      std::cout << " A Triplet : " << fHCa << "\n";

      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetClusterRadius(fClusterRadius);
      dynamic_cast<AtPATTERN::AtTrackFinderTC *>(fPRA)->SetClusterDistance(fClusterDistance);

      std::cout << " Track finder - Parameters for clusterization "
                << "\n";
      std::cout << " Cluster radius " << fClusterRadius << "\n";
      std::cout << " Cluster distance " << fClusterDistance << "\n";

   } else if (fPRAlgorithm == 1) {
      LOG(info) << "Using RANSAC algorithm";

   } else if (fPRAlgorithm == 2) {
      LOG(info) << "Using Hough transform algorithm";
      // fPSA = new AtPSAProto();
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
   if (ioMan == nullptr) {
      LOG(error) << "Cannot find RootManager!";
      return kERROR;
   }

   fEventHArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fInputBranchName));
   if (fEventHArray == nullptr) {
      LOG(error) << "Cannot find AtEvent array!";
      return kERROR;
   }

   ioMan->Register(fOutputBranchName, "AtTPC", &fPatternEventArray, kIsPersistence);

   return kSUCCESS;
}

void AtPRAtask::Exec(Option_t *option)
{
   LOG(debug) << "Exec of AtPRAtask";

   fPatternEventArray.Delete();

   if (fEventHArray->GetEntriesFast() == 0)
      return;

   AtEvent &event = *(dynamic_cast<AtEvent *>(fEventHArray->At(0))); // TODO: Make sure we are not copying
   auto &hitArray = event.GetHits();

   std::cout << "  -I- AtPRAtask -  Event Number :  " << event.GetEventID() << "\n";

   try {

      if (hitArray.size() > fMinNumHits && hitArray.size() < fMaxNumHits) {
         auto patternEvent = fPRA->FindTracks(event);
         new (fPatternEventArray[0]) AtPatternEvent(std::move(*patternEvent));
      }

   } catch (std::runtime_error e) {
      std::cout << "Analyzation failed! Error: " << e.what() << std::endl;
   }

   // fEvent  = (AtEvent *) fEventHArray -> At(0);
}

void AtPRAtask::Finish()
{
   LOG(debug) << "Finish of AtPRAtask";
}
