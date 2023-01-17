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
class AtTabInfoHiRAEVT : public AtTabInfoBase {
protected:
   std::unique_ptr<TTreeReader> fReader;
   std::unique_ptr<TTreeReaderValue<T>> fDetectorReader;

   TString fTree;
   TString fBranchName;

   T *fDetector;

public:
   AtTabInfoHiRAEVT(TString branchName) : AtTabInfoBase(), fBranchName(branchName) {}
   void Init() override
   {
      fReader = std::make_unique<TTreeReader>(fTree);
      fDetectorReader = std::make_unique<TTreeReaderValue<T>>(*fReader, fBranchName);
   }
   void Update() override
   { // Get the current entry of the tree

      auto entry = FairRootManager::Instance()->GetInTree()->GetReadEntry();
      fReader->SetEntry(entry);
      fDetector = fDetectorReader->Get();
   }

   void Update(DataHandling::Subject *changedSubject) override {}

   T *GetInfo()
   {
      Update();
      return fDetector;
   }

   void SetTree(TString name) { fTree = name; }
};

#endif
