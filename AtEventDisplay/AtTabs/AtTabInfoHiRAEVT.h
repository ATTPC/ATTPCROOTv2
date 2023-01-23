#ifndef ATTABINFOHIRAEVT_H
#define ATTABINFOHIRAEVT_H

#include "AtDataObserver.h"         // for Observer
#include "AtTabInfo.h"              // for AtTabInfoBase
#include "AtViewerManagerSubject.h" // for AtTreeEntry

#include <TString.h>     // for TString
#include <TTreeReader.h> // for TTreeReader

#include <memory> // for unique_ptr, make_unique
#include <string>

namespace DataHandling {
class AtSubject;
}
template <typename T>
class TTreeReaderValue;

template <typename T>
class AtTabInfoHiRAEVT : public AtTabInfoBase, public DataHandling::AtObserver {
protected:
   std::unique_ptr<TTreeReader> fReader{nullptr};
   std::unique_ptr<TTreeReaderValue<T>> fDetectorReader{nullptr};

   TString fTree;
   TString fBranchName;

   DataHandling::AtTreeEntry &fEntryNumber;
   T *fDetector{nullptr};

public:
   AtTabInfoHiRAEVT(TString tree, TString branchName, DataHandling::AtTreeEntry &entryNumber)
      : AtTabInfoBase(), fTree(tree), fBranchName(branchName), fEntryNumber(entryNumber)
   {
      fEntryNumber.Attach(this);
   }
   ~AtTabInfoHiRAEVT() { fEntryNumber.Detach(this); }

   void Init() override
   {
      fReader = std::make_unique<TTreeReader>(fTree);
      fDetectorReader = std::make_unique<TTreeReaderValue<T>>(*fReader, fBranchName);
   }

   void Update(DataHandling::AtSubject *changedSubject) override
   {
      if (changedSubject == &fEntryNumber)
         Update();
   }

   T *GetInfo() { return fDetector; }

   void SetTree(TString name) { fTree = name; }
   std::string GetDefaultName() override { return T::Class_Name(); }

protected:
   void Update() override
   {
      fReader->SetEntry(fEntryNumber.Get());
      fDetector = fDetectorReader->Get();
   }
};

#endif
