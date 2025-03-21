#ifndef AT3DBRAGGCURVETASK_H
#define AT3DBRAGGCURVETASK_H

#include "At3DBraggCurveHandler.h"
#include "At3DBraggFitter.h"

#include <FairTask.h> // for FairTask, InitStatus

#include <Rtypes.h> // for Int_t, Bool_t, Double_t, THashConsistencyHolder
#include <TClonesArray.h>
#include <TString.h> // for TString

class AtRawEvent;
class AtPatternEvent;
class TBuffer;
class TClass;
class TMemberInspector;

class At3DBraggCurveTask : public FairTask {
private:
   TString fRawEventBranchName;
   TString fPatternEventBranchName;
   TString fOutputPatternEventBranchName;

   TClonesArray *fRawEventArray{nullptr};
   TClonesArray *fPatternEventArray{nullptr};
   TClonesArray fOutputPatternEventArray;

   AtRawEvent *fRawEvent{};
   AtPatternEvent *fPatternEvent{};

   Bool_t kIsPersistence;
   Double_t fChargeThres{-1};

   At3DBraggCurveHandler *f3DBraggHandler{nullptr};
   std::unique_ptr<AtFITTER::At3DBraggFitter> f3DBraggFitter{nullptr};

public:
   At3DBraggCurveTask();
   ~At3DBraggCurveTask();

   void SetRawEventBranch(TString branchName) { fRawEventBranchName = branchName; }
   void SetPatternEventBranch(TString branchName) { fPatternEventBranchName = branchName; }
   void SetOutputPatternEventBranch(TString branchName) { fOutputPatternEventBranchName = branchName; }
   void SetPersistence(Bool_t value = kTRUE) { kIsPersistence = value; }

   void SetChargeThreshold(Double_t value) { fChargeThres = value; }

   void Set3DBraggCurveFitter(std::unique_ptr<AtFITTER::At3DBraggFitter> fitter) { f3DBraggFitter = std::move(fitter); }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;

   ClassDefOverride(At3DBraggCurveTask, 1);
};

#endif
