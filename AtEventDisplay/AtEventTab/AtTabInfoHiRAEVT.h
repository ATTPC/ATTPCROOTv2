#ifndef ATTABINFOHIRAEVT_H
#define ATTABINFOHIRAEVT_H

#include "AtTabInfoBase.h"

#include <FairRootManager.h>

#include <Rtypes.h> // for ClassDefOverride
#include <TString.h>
#include <TTree.h> // for TTree
#include <TTreeReader.h>
//#include <TTreeReaderValue.h>

#include <memory> // for unique_ptr, make_unique
class TBuffer;
class TClass;
class TMemberInspector;
template <typename T>
class TTreeReaderValue;
// class TTreeReader;

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
      auto entry = FairRootManager::Instance()->GetInTree()->GetEntry();
      fReader->SetEntry(entry);
      fDetector = fDetectorReader->Get();
   }

   T *GetInfo() { return fDetector; }

   void SetTree(TString name) { fTree = name; }

   ClassDefOverride(AtTabInfoHiRAEVT, 1);
};

#endif
