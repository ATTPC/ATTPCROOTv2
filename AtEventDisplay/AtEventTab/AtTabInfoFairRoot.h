#ifndef ATTABINFOFAIRROOT_H
#define ATTABINFOFAIRROOT_H

#include "AtTabInfoBase.h"

#include <FairLogger.h> // for Logger, LOG
#include <FairRootManager.h>

#include <TClonesArray.h>
#include <TObject.h> // for TObject
#include <TString.h>

#include <memory> // for allocator

/**
 * @brief Class for tracking the information from a branch of the FairRoot source tree.
 *
 * Templated class, for an AtRawEvent it would be instantiated like AtTabInfo<AtRawEvent>.
 * Switched to template to make adding additional info types stored on the TTree trivial to add.
 */
template <typename T>
class AtTabInfoFairRoot : public AtTabInfoBase {

protected:
   TString fBranchName;
   TClonesArray *fEventArray{};
   T *fInfo;

public:
   AtTabInfoFairRoot() : AtTabInfoFairRoot(T::Class_Name()) {}
   AtTabInfoFairRoot(TString branchName) : AtTabInfoBase(), fBranchName(branchName) {}

   void Init() override
   {

      fEventArray = dynamic_cast<TClonesArray *>(FairRootManager::Instance()->GetObject(fBranchName));

      constexpr auto cNORMAL = "\033[0m";
      constexpr auto cGREEN = "\033[1;32m";
      if (fEventArray)
         LOG(INFO) << cGREEN << "Found branch " << fBranchName << " containing " << T::Class_Name() << "." << cNORMAL;
   }

   void Update() override { fInfo = dynamic_cast<T *>(fEventArray->At(0)); }
   void SetBranch(TString branchName) { fBranchName = branchName; }
   T *GetInfo() { return fInfo; }
};

#endif
