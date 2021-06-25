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

// FairRoot classes
#include "FairTask.h"

#include <vector>

// HiRAEVT classes
class HTTimestamp;

// ROOT classes
class TClonesArray;
class TChain;
class TGraph;

// ATTPCROOT classes
class AtRawEvent;

class AtLinkDAQTask : public FairTask {

private:
   // Info for AT-TPC Tree
   TClonesArray *fInputEventArray; // AtRawEvent
   TString fInputBranchName;       // Name if AtRawEvent branch
   AtRawEvent *fRawEvent;

   // Info for HiRAEVT Tree input
   TChain *evtTree;
   TString fEvtTimestampName;
   HTTimestamp *fEvtTS;

   // Info for output HiRAEVT tree
   TString fEvtOutputFileName;
   TFile *fEvtOutputFile;
   TTree *fEvtOutputTree;

   // Variables used during merging
   ULong64_t fEvtTreeIndex;
   ULong64_t fOldEvtTimestamp;
   ULong64_t fEvtTimestamp;
   Double_t fIntervalEvt;

   ULong64_t fTpcTreeIndex;
   std::vector<ULong64_t> fOldTpcTimestamp;
   std::vector<ULong64_t> fTpcTimestamp;
   Double_t fIntervalTpc;

   Double_t fDifferenceOffset;

   // Input parameters
   Double_t fSearchMean; // This is the ratio of clock frequencies
   Double_t fSearchRadius;
   Double_t fCorruptedSearchRadius;
   Int_t fTpcTimestampIndex;

   Bool_t kPersistent;
   Bool_t kFirstEvent;
   Bool_t kFillEvt;
   Bool_t kCorruptedTimestamp;

   // diagnostic graphs to write in HiRAEVT output file
   std::vector<std::vector<double>> fGrDataRatio;
   std::vector<std::vector<double>> fGrDataAbs;

   Double_t GetScaledInterval(ULong64_t intervalEvt, ULong64_t intervalTpc);

   void DoFirstEvent();
   void UpdateTimestamps();
   void ResetFlags();
   void Fill();
   Int_t CheckMatch();
   bool ExtraEvtEvent();

public:
   AtLinkDAQTask();
   ~AtLinkDAQTask();

   bool SetInputTree(TString fileName, TString treeName);
   bool AddInputTree(TString fileName);
   void SetEvtTimestamp(TString name) { fEvtTimestampName = name; }
   void SetTpcTimestampIndex(Int_t index) { fTpcTimestampIndex = index; }
   void SetEvtOutputFile(TString fileName) { fEvtOutputFileName = fileName; }
   void SetPersistance(Bool_t val) { kPersistent = val; }

   void SetSearchMean(Double_t mean) { fSearchMean = mean; }
   void SetSearchRadius(Double_t radius) { fSearchRadius = radius; }
   void SetCorruptedSearchRadius(Double_t radius) { fCorruptedSearchRadius = radius; }

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
