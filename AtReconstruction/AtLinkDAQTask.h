#ifndef ATLINKDAQTASK_H
#define ATLINKDAQTASK_H
/*
 * Task to link runs in attpcroot and HiRAEVT
 * requires that the AT-TPC root code was build against the HiRAEVT code
 * This doesn't modify the tree at all, but creates a new tree holding a copy
 * of the detectors in the passed HiRAEVT tree, which is added as a friend to
 * the AT-TPC root tree. That way this only has to be run per run and the TPC tree
 * can be recreated without having to re-run this code.
 *
 * This links the DAQs by looking at the reduced interval between events, where the reduced interval i
 * intervalTPC/intervalNSCL. This works even when the clocks are different frequencies
 *
 *
 * As an input it takes some mean and radius to search around, and a histogram of the reduced interval
 * is saved.
 *
 * Requires use of the new run type AtRunAna, it extends FairRunAna but exposes some additional
 * behavior
 *
 * Adam Anthony 5/27/21
 *
 */

#include <TChain.h>
#include <TString.h>
#include <TTree.h>
// FairRoot classes
#include <FairTask.h>

#include <Rtypes.h>

#include <memory>
#include <vector>

// HiRAEVT classes
class HTTimestamp;
// ROOT classes
class TClonesArray;
// ATTPCROOT classes
class AtRawEvent;
class TFile;

class AtLinkDAQTask : public FairTask {

private:
   // Info for AT-TPC Tree
   TClonesArray *fInputEventArray{};       // AtRawEvent
   TString fInputBranchName{"AtRawEvent"}; // Name if AtRawEvent branch
   AtRawEvent *fRawEvent{nullptr};

   // Info for HiRAEVT Tree input
   std::unique_ptr<TChain> evtTree{nullptr};
   TString fEvtTimestampName;
   HTTimestamp *fEvtTS{nullptr};

   // Info for output HiRAEVT tree
   TString fEvtOutputFileName{""};
   TFile *fEvtOutputFile{};
   TTree *fEvtOutputTree{};

   // Variables used during merging
   ULong64_t fEvtTreeIndex{0};
   ULong64_t fOldEvtTimestamp{0};
   ULong64_t fEvtTimestamp{0};
   Double_t fIntervalEvt{0};

   ULong64_t fTpcTreeIndex{0};
   std::vector<ULong64_t> fOldTpcTimestamp;
   std::vector<ULong64_t> fTpcTimestamp;
   Double_t fIntervalTpc{0};

   Double_t fDifferenceOffset{0};

   // Input parameters
   Double_t fSearchMean{0}; // This is the ratio of clock frequencies
   Double_t fSearchRadius{0};
   Double_t fCorruptedSearchRadius{0};
   Int_t fTpcTimestampIndex{0};

   Bool_t kPersistent{false};
   Bool_t kFirstEvent{true};
   Bool_t kFillEvt{};
   Bool_t kCorruptedTimestamp{};

   // diagnostic graphs to write in HiRAEVT output file
   std::vector<std::vector<double>> fGrDataRatio;
   std::vector<std::vector<double>> fGrDataAbs;

   Double_t GetScaledInterval(ULong64_t intervalEvt, ULong64_t intervalTpc);

   void DoFirstEvent();
   bool UpdateTimestamps();
   void ResetFlags();
   void Fill();
   Int_t CheckMatch();
   bool ExtraEvtEvent();

public:
   AtLinkDAQTask() = default;
   ~AtLinkDAQTask() = default;

   bool SetInputTree(TString fileName, TString treeName);
   bool AddInputTree(TString fileName);
   void SetEvtTimestamp(TString name) { fEvtTimestampName = name; }
   void SetTpcTimestampIndex(Int_t index) { fTpcTimestampIndex = index; }
   void SetEvtOutputFile(TString fileName) { fEvtOutputFileName = fileName; }
   void SetPersistance(Bool_t val) { kPersistent = val; }
   void SetInputBranchName(TString name) { fInputBranchName = name; }

   void SetSearchMean(Double_t mean) { fSearchMean = mean; }
   void SetSearchRadius(Double_t radius) { fSearchRadius = radius; }
   void SetCorruptedSearchRadius(Double_t radius) { fCorruptedSearchRadius = radius; }

   const TChain *GetInChain() { return evtTree.get(); }
   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;
   virtual void Finish() override; // called at end of run
   virtual void FinishEvent() override
   {
      if (kFillEvt)
         fEvtOutputTree->Fill();
   } // Called at the end of an event
};
#endif //#define ATLINKDAQTASK_H
