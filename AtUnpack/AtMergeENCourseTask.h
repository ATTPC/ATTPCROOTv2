#ifndef ATMERGEENCOURSETASK_H
#define ATMERGEENCOURSETASK_H

#include "AtENCourseEvent.h"
#include "AtPPACPair.h"

#include <FairTask.h>

#include <Rtypes.h>
#include <TClonesArray.h>
#include <TFile.h>
#include <TTree.h>

#include <memory>
#include <string>

class AtMergeENCourseTask : public FairTask {
private:
   typedef struct madc32_data {
      unsigned int adc[32];
      unsigned long int counter[1];
   } madc32_data;

   TString fInputFileName;
   TString fOuputBranchName{"AtENCourseEvent"};
   TString fRawEventBranchName{"AtRawEvent"};
   Bool_t fIsPersistent = true;

   TClonesArray fOutputENCourseEventArray;
   TClonesArray *fRawEventArray;

   std::unique_ptr<TFile> fENCourseFile{nullptr};
   std::unique_ptr<TTree> fENCourseTree{nullptr};
   Int_t eve;
   Float_t ppac_pos_cal[4][2];
   Int_t rf[4];
   Int_t ref_tdc;
   madc32_data madc;
   ULong64_t fLastTreeEntryNum{};

   ULong64_t fLastENTS{};
   ULong64_t fLastATTPCTS{};
   Int_t fMaxDeltaTimeDifference{5}; // [us]

   Double_t fF2PPACsDistance{500}; // [mm]
   Double_t fF3PPACsDistance{500}; // [mm]

public:
   AtMergeENCourseTask();
   ~AtMergeENCourseTask() = default;

   void SetInputFileName(TString filename) { fInputFileName = filename; }
   void SetOuputBranchName(TString branchName) { fOuputBranchName = branchName; }
   void SetPersistence(Bool_t value) { fIsPersistent = value; }

   void SetMaxDeltaTimeDifference(Int_t value) { fMaxDeltaTimeDifference = value; }
   void SetF2PPACsDistance(Double_t value) { fF2PPACsDistance = value; }
   void SetF3PPACsDistance(Double_t value) { fF3PPACsDistance = value; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;

   void CloseENRootFile();

   ClassDefOverride(AtMergeENCourseTask, 1);
};

#endif
