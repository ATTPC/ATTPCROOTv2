#ifndef ATMERGEENCOURSETASK_H
#define ATMERGEENCOURSETASK_H

#include "AtENCourseEvent.h"
#include "AtPPACPair.h"

#include <FairTask.h>

#include <Rtypes.h>
#include <TClonesArray.h>
#include <TFile.h>

#include <memory>
#include <string>

class AtMergeENCourseTask : public FairTask {
private:
   TString fInputFileName;
   TString fOuputBranchName{"AtENCourseEvent"};
   Bool_t fIsPersistent = true;

   TClonesArray fOutputENCourseEventArray;

   std::unique_ptr<TFile> fENCourseFile{nullptr};
   std::unique_ptr<TTree> fENCourseTree{nullptr};
   Int_t eve;
   Float_t ppac_pos_cal[4][2];
   ULong64_t fLastTreeEntryNum{};

   Double_t fF2PPACsDistance{500}; // [mm]
   Double_t fF3PPACsDistance{500}; // [mm]

public:
   AtMergeENCourseTask();
   ~AtMergeENCourseTask() = default;

   void SetInputFileName(TString filename) { fInputFileName = filename; }
   void SetOuputBranchName(TString branchName) { fOuputBranchName = branchName; }
   void SetPersistence(Bool_t value) { fIsPersistent = value; }

   void SetF2PPACsDistance(Double_t value) { fF2PPACsDistance = value; }
   void SetF3PPACsDistance(Double_t value) { fF3PPACsDistance = value; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;

   ClassDefOverride(AtMergeENCourseTask, 1);
};

#endif
