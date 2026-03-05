#include "AtMergeENCourseTask.h"

#include "AtRawEvent.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairTask.h>

#include <Math/Point2D.h>
#include <Math/Point2Dfwd.h>

#include <utility>

using XYPoint = ROOT::Math::XYPoint;

ClassImp(AtMergeENCourseTask);

AtMergeENCourseTask::AtMergeENCourseTask() : fOutputENCourseEventArray(TClonesArray("AtENCourseEvent", 1)) {}

InitStatus AtMergeENCourseTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "Cannot find RootManager!";
      return kERROR;
   }

   fRawEventArray = dynamic_cast<TClonesArray *>(ioMan->GetObject(fRawEventBranchName));
   if (fRawEventArray == nullptr) {
      LOG(error) << "AtRawEvent branch was not found. The merger can not check the time stamp synchronization!";
      return kERROR;
   }

   fENCourseFile = std::make_unique<TFile>(fInputFileName, "READ");
   if (fENCourseFile->IsZombie()) {
      LOG(error) << "Could not open ROOT file " << fInputFileName << "!";
      return kERROR;
   }
   LOG(info) << "ROOT file " << fInputFileName << " opened successfully.";

   fENCourseTree = std::unique_ptr<TTree>((TTree *)fENCourseFile->Get("tree"));
   fENCourseTree->SetBranchAddress("eve", &eve);
   fENCourseTree->SetBranchAddress("ppac_pos_cal", ppac_pos_cal);
   fENCourseTree->SetBranchAddress("rf", rf);
   fENCourseTree->SetBranchAddress("ref_tdc", &ref_tdc);
   fENCourseTree->SetBranchAddress("madc", &madc);

   ioMan->Register(fOuputBranchName, "ENCourse", &fOutputENCourseEventArray, fIsPersistent);

   return kSUCCESS;
}

void AtMergeENCourseTask::Exec(Option_t *opt)
{
   AtENCourseEvent *ENCourseEvent = dynamic_cast<AtENCourseEvent *>(fOutputENCourseEventArray.ConstructedAt(0));
   AtRawEvent *rawEvent = dynamic_cast<AtRawEvent *>(fRawEventArray->At(0));

   if (fLastTreeEntryNum >= fENCourseTree->GetEntries()) {
      ENCourseEvent->SetIsGood(false);
      return;
   }

   fENCourseTree->GetEntry(fLastTreeEntryNum++);
   LOG(info) << "Merging ENCourse event " << eve << " with timestamp " << madc.counter[0]
             << ". The corresponding AtRawEvent has timestamp " << rawEvent->GetTimestamp(1) << " with the 1MHz clock.";
   ENCourseEvent->SetEventID(eve);
   ENCourseEvent->SetTimestamp(madc.counter[0]);

   auto deltaTEN = madc.counter[0] - fLastENTS;
   auto deltaTATTPC = rawEvent->GetTimestamp(1) - fLastATTPCTS;
   fLastENTS = madc.counter[0];
   fLastATTPCTS = rawEvent->GetTimestamp(1);

   if (std::abs(Long64_t(deltaTEN - deltaTATTPC)) > fMaxDeltaTimeDifference) {
      LOG(warning)
         << " This ENCourse event is not time correlated with its respective AtRawEvent. Marking it as not good!";
      ENCourseEvent->SetIsGood(false);
      return;
   }

   XYPoint F2EntrancePoint(ppac_pos_cal[0][0], ppac_pos_cal[0][1]);
   XYPoint F2ExitPoint(ppac_pos_cal[1][0], ppac_pos_cal[1][1]);
   std::unique_ptr<AtPPACPair> F2PPACs = std::make_unique<AtPPACPair>(F2EntrancePoint, F2ExitPoint, fF2PPACsDistance);

   XYPoint F3EntrancePoint(ppac_pos_cal[2][0], ppac_pos_cal[2][1]);
   XYPoint F3ExitPoint(ppac_pos_cal[3][0], ppac_pos_cal[3][1]);
   std::unique_ptr<AtPPACPair> F3PPACs = std::make_unique<AtPPACPair>(F3EntrancePoint, F3ExitPoint, fF3PPACsDistance);

   ENCourseEvent->SetF2PPACs(std::move(F2PPACs));
   ENCourseEvent->SetF3PPACs(std::move(F3PPACs));
   for (int i = 0; i < 4; i++)
      ENCourseEvent->SetRFToF(rf[i], i);
   ENCourseEvent->SetTDCRef(ref_tdc);
   ENCourseEvent->SetIsGood(true);
}

void AtMergeENCourseTask::CloseENRootFile()
{
   if (fENCourseFile)
      fENCourseFile->Close();
}
