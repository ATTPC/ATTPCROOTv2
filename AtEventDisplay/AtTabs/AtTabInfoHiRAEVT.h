#ifndef ATTABINFOHIRAEVT_H
#define ATTABINFOHIRAEVT_H

#include "AtTabInfo.h"

#include <FairRootManager.h>

#include <TString.h>
#include <TTree.h> // for TTree
#include <TTreeReader.h>

#include <memory> // for unique_ptr, make_unique
template <typename T>
class TTreeReaderValue;

template <typename T>
class AtTabInfoHiRAEVT : public AtTabInfoBase, public DataHandling::Observer {
protected:
   std::unique_ptr<TTreeReader> fReader{nullptr};
   std::unique_ptr<TTreeReaderValue<T>> fDetectorReader{nullptr};

   TString fTree;
   TString fBranchName;

   DataHandling::AtTreeEntry *fEntryNumber;
   T *fDetector{nullptr};

public:
   AtTabInfoHiRAEVT(TString tree, TString branchName, DataHandling::AtTreeEntry *entryNumber)
      : AtTabInfoBase(), fTree(tree), fBranchName(branchName), fEntryNumber(entryNumber)
   {
      fEntryNumber->Attach(this);
   }
   ~AtTabInfoHiRAEVT() { fEntryNumber->Detach(this); }

   void Init() override
   {
      fReader = std::make_unique<TTreeReader>(fTree);
      fDetectorReader = std::make_unique<TTreeReaderValue<T>>(*fReader, fBranchName);
   }

   void Update(DataHandling::Subject *changedSubject) override
   {
      if (changedSubject == fEntryNumber)
         Update();
   }

   T *GetInfo() { return fDetector; }

   void SetTree(TString name) { fTree = name; }

protected:
   void Update() override
   {
      fReader->SetEntry(fEntryNumber->Get());
      fDetector = fDetectorReader->Get();
   }
};

#endif
