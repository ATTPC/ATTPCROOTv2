#ifndef ATTABINFO_H
#define ATTABINFO_H

#include <FairLogger.h> // for Logger, LOG
#include <FairRootManager.h>

#include <TClonesArray.h>
#include <TObject.h> // for TObject
#include <TString.h>

#include <map>
#include <memory> // for allocator
#include <memory> // for unique_ptr
#include <string>

/**
 * @defgroup TabData Data for tabs
 *
 * Classes used to provide access to and update the data used by tabs in the visualizer. There are two main classes at
 the moment:

 * AtTabInfoFairRoot which provides access to classes through the FairRoot source file.
 * AtTabInfoHiRAEVT which provides access to classes stored in a HiRAEVT stle TTree.
 * A S800 class will also have to be added.
 *
 * These all follow a composition pattern (similar to AtPad). Each tab owns a single AtTabInfo which
 * can itself contain any number at AtTabInfo-style classes.
 */

/**
 * @brief Interface for AtTabInfo classes.
 * @ingroup TabData
 */
class AtTabInfoBase {
public:
   AtTabInfoBase() = default;
   virtual ~AtTabInfoBase() = default;

   /**
    * @brief setup how to access this info from its data source.
    */
   virtual void Init() = 0;
   /**
    * @brief update the data this holds from its source.
    */
   virtual void Update() = 0;
};

/**
 * @brief Contains all the data needed by an AtTab.
 * @ingroup TabData
 */
class AtTabInfo : public AtTabInfoBase {
protected:
   std::map<std::string, std::unique_ptr<AtTabInfoBase>> fInfoAugments;

public:
   AtTabInfo() = default;
   void Init() override;
   void Update() override;

   AtTabInfoBase *AddAugment(std::string name, std::unique_ptr<AtTabInfoBase> augment);
   AtTabInfoBase *ReplaceAugment(std::string name, std::unique_ptr<AtTabInfoBase> augment);
   AtTabInfoBase *GetAugment(std::string name);
};

/**
 * @brief Class for tracking the information from a branch of the FairRoot source tree.
 *
 * Templated class, for an AtRawEvent it would be instantiated like AtTabInfo<AtRawEvent>.
 * Switched to template to make adding additional info types stored on the TTree trivial to add.
 * @ingroup TabData
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
