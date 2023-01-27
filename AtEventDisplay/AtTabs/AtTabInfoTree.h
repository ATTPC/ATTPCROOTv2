#ifndef ATTABINFOTREE_H
#define ATTABINFOTREE_H

#include "AtDataObserver.h"         // for Observer
#include "AtTabInfo.h"              // for AtTabInfoBase
#include "AtViewerManagerSubject.h" // for AtTreeEntry

#include <TFile.h>
#include <TString.h>          // for TString
#include <TTree.h>            // for TTree
#include <TTreeReader.h>      // for TTreeReader
#include <TTreeReaderValue.h> // for TTreeReaderValue

#include <memory> // for unique_ptr, make_unique
#include <string>

namespace DataHandling {
class AtSubject;
}

/**
 * @brief Tab Info object for handling a non-FairRoot TTree.
 *
 * Esentially takes the place of FairRootManager for non FairRoot TTrees. It is suggested (possibly required) that each
 * tree object is only created once, and then copied to other tabs or objects. Otherwise memory issues may arrise. They
 * should hang around as shared pointers so they survive until everyone using the tree is done.
 * @ingroup TabData
 */
class AtTabInfoTree : public AtTabInfoBase, public DataHandling::AtObserver {
protected:
   TTreeReader fReader;
   std::unique_ptr<TFile> fFile{nullptr}; //< Pointer for if we are responsible for the file w/ tree
   DataHandling::AtTreeEntry &fEntryNumber;

public:
   AtTabInfoTree(TString tree, TString fileName, DataHandling::AtTreeEntry &entryNumber);
   AtTabInfoTree(TTree *tree, DataHandling::AtTreeEntry &entryNumber);
   ~AtTabInfoTree() { fEntryNumber.Detach(this); }

   std::string GetDefaultName() override { return fReader.GetTree()->GetName(); }
   void Update(DataHandling::AtSubject *changedSubject) override;

   TTreeReader &GetReader() { return fReader; }
   TTree *GetTree() { return fReader.GetTree(); }
};

/**
 *@brief Describes a branch stored a TTree.
 *
 * TTree and entry number themsleves are handled by an AtTabInfoTree object.
 *
 * @ingroup TabData
 */
template <typename T>
class AtTabInfoBranch : public AtTabInfoBase {
protected:
   TTreeReaderValue<T> fDetector;
   std::shared_ptr<AtTabInfoTree> fTree;

public:
   AtTabInfoBranch(std::shared_ptr<AtTabInfoTree> tree, TString branchName)
      : AtTabInfoBase(), fDetector(tree->GetReader(), branchName), fTree(tree)
   {
   }

   T *GetInfo() { return fDetector->Get(); }

   std::string GetDefaultName() override { return T::Class_Name(); }
};

#endif
