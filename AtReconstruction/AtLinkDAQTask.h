#ifndef ATLINKDAQ_H
#define ATLINKDAQ_H
/*
 * Task to link runs in attpcroot and HiRAEVT
 * requires that the AT-TPC root code was build against the HiRAEVT code
 * This doesn't modify the tree at all, but creates a new tree holding a copy
 * of the detectors in the passed HiRAEVT tree, which is added as a friend to
 * the AT-TPC root tree. That way this only has to be run per run and the TPC tree
 * can be recreated without having to re-run this code.
 *
 * This links the DAQs by looking at the reduced interval between events, where the reduced interval is
 * |intervalTPC - intervalNSCL|/intervalNSCL. This works even when the clocks are different frequencies
 * As an input it takes some mean and radius to search around, and a histogram of the reduced interval
 * is saved.
 *
 * Adam Anthony 5/27/21
 *
 */

// FairRoot classes
#include "FairTask.h"

// HiRAEVT classes
#include "HTTimestamp.h"

// ROOT classes
class TClonesArray;
class TChain;
class TTreeReader;
#include "TTreeReaderValue.h"

class AtLinkDAQTask : public FairTask {

private:
   TClonesArray *fInputEventArray; // AtRawEvent
   TClonesArray *fMCPointArray;    // AtRawEvent
   TChain *evtTree;
   TTreeReader *evtReader;
   TTreeReaderValue<HTTimestamp> *evtTimestamp; //!

   Double_t fSearchMean;
   Double_t fSearchRadius;
   ULong64_t fOldTpcTimestamp;
   ULong64_t fOldEvtTimestamp;
   Int_t fTpcTimestampIndex;

public:
   AtLinkDAQTask();
   ~AtLinkDAQTask();

   void SetInputTree(TString fileName, TString treeName);
   void SetEvtTimestamp(TString name);
   void SetTpcTimestampIndex(Int_t index) { fTpcTimestampIndex = index; }

   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;
   virtual void Finish() override; // called at end of run
};
#endif //#ifndef ATFILTERTASK_H
