#ifndef ATCOPYAUXTREETASK_H
#define ATCOPYAUXTREETASK_H
/*
 * Task to copy the tree structure of an auxilary input root file to an output root file
 * when using AtDataReductionTask with ATTPC files.
 *
 * An example would be when sorting ATTPC events with an associated root file from
 * a separate DAQ system (such as HiRAEVT). This task would copy only the
 * corresponding events to a new output file, ensuring parity between the root files
 * from the two DAQs.
 *
 */

#include <TChain.h>
#include <TString.h>
#include <TTree.h>
// FairRoot classes
#include <FairTask.h>

#include <Rtypes.h>

#include <memory>

// ROOT classes
class TClonesArray;
// ATTPCROOT classes
class TFile;

class AtCopyAuxTreeTask : public FairTask {

private:
   // Info for AT-TPC Tree
   TClonesArray *fCheckEventArray{};            // AtBaseEvent
   TString fCheckEventBranchName{"AtRawEvent"}; // Name of AtBaseEvent branch used to check if event is good

   // Info for output tree
   TString fOutputFileName{""};
   TFile *fOutputFile{};
   TTree *fOutputTree{};

   std::unique_ptr<TChain> fInputTree{nullptr};

public:
   AtCopyAuxTreeTask() = default;
   ~AtCopyAuxTreeTask() = default;

   /**
    * Initializes a TChain with treeName and adds the file to the TChain
    */
   bool SetInputTree(TString fileName, TString treeName);
   /**
    * Adds additional files to the TChain
    */
   bool AddInputFile(TString fileName);
   /**
    * Sets the name of the output root file
    */
   void SetOutputFile(TString fileName) { fOutputFileName = fileName; }
   /**
    * Sets the name of the AtRawEvent branch that is monitored for good events
    */
   void SetCheckEventBranch(TString name) { fCheckEventBranchName = name; }

   /**
    * Returns a pointer to the TChain object for the input tree
    */
   const TChain *GetInChain() { return fInputTree.get(); }
   /**
    * Retruns a pointer to the output root file
    */
   TFile *GetOutFile() { return fOutputFile; }
   virtual InitStatus Init() override;
   virtual void Exec(Option_t *opt) override;
   virtual void Finish() override; // called at end of run
};
#endif // #define AtCopyAuxTreeTASK_H
