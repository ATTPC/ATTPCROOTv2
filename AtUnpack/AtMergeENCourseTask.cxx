#include "AtMergeENCourseTask.h"

#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairTask.h>

#include <Math/Point2D.h>
#include <Math/Point2Dfwd.h>

#include <utility>

using XYPoint = ROOT::Math::XYPoint;

ClassImp(AtMergeENCourseTask);

AtMergeENCourseTask::AtMergeENCourseTask()
   : fOutputENCourseEventArray(TClonesArray("AtENCourseEvent", 1))
{
}

InitStatus AtMergeENCourseTask::Init()
{
   FairRootManager *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "Cannot find RootManager!";
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

   ioMan->Register(fOuputBranchName, "ENCourse", &fOutputENCourseEventArray, fIsPersistent);

   return kSUCCESS;
}

void AtMergeENCourseTask::Exec(Option_t *opt)
{
   AtENCourseEvent *ENCourseEvent = dynamic_cast<AtENCourseEvent *>(fOutputENCourseEventArray.ConstructedAt(0));

   if (fLastTreeEntryNum >= fENCourseTree->GetEntries()) {
      ENCourseEvent->SetIsGood(false);
      return;
   }

   fENCourseTree->GetEntry(fLastTreeEntryNum++);
   LOG(info) << "Merging ENCourse event " << eve;

   XYPoint F2EntrancePoint(ppac_pos_cal[0][0], ppac_pos_cal[0][1]);
   XYPoint F2ExitPoint(ppac_pos_cal[1][0], ppac_pos_cal[1][1]);
   std::unique_ptr<AtPPACPair> F2PPACs = std::make_unique<AtPPACPair>(F2EntrancePoint, F2ExitPoint, fF2PPACsDistance);
   //F2PPACs->SetTimeOfFlight(TOFF2U?, 0);
   //F2PPACs->SetTimeOfFlight(TOFF2D?, 1);

   XYPoint F3EntrancePoint(ppac_pos_cal[2][0], ppac_pos_cal[2][1]);
   XYPoint F3ExitPoint(ppac_pos_cal[3][0], ppac_pos_cal[3][1]);
   std::unique_ptr<AtPPACPair> F3PPACs = std::make_unique<AtPPACPair>(F3EntrancePoint, F3ExitPoint, fF3PPACsDistance);
   //F3PPACs->SetTimeOfFlight(TOFF3U?, 0);
   //F3PPACs->SetTimeOfFlight(TOFF3D?, 1);

   ENCourseEvent->SetF2PPACs(std::move(F2PPACs));
   ENCourseEvent->SetF3PPACs(std::move(F3PPACs));
   ENCourseEvent->SetEventID(eve);
   //ENCourseEvent->SetTimestamp(ENTS?);

}
