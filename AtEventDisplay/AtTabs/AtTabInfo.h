#ifndef ATTABINFO_H
#define ATTABINFO_H

#include "AtDataObserver.h"
#include "AtDataSubject.h"
#include "AtViewerManagerSubject.h"

#include <FairLogger.h> // for Logger, LOG
#include <FairRootManager.h>

#include <TClonesArray.h>
#include <TObject.h> // for TObject
#include <TString.h>

#include <map>
#include <memory> // for allocator
#include <string>

/**
 * @defgroup TabData Data for tabs
 *
 * Classes used to provide access to and update the data used by tabs in the visualizer. There are
 two main classes at the moment:

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
    * @brief Default name for info type.
    */
   virtual std::string GetDefaultName() = 0;

protected:
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
   ~AtTabInfo() = default;

   void Init() override;

   std::string GetDefaultName() override { return "AtTabInfo"; }

   AtTabInfoBase *AddAugment(std::unique_ptr<AtTabInfoBase> augment);
   AtTabInfoBase *AddAugment(std::unique_ptr<AtTabInfoBase> augment, std::string name);

   AtTabInfoBase *ReplaceAugment(std::unique_ptr<AtTabInfoBase> augment);
   AtTabInfoBase *ReplaceAugment(std::unique_ptr<AtTabInfoBase> augment, std::string name);

   AtTabInfoBase *GetAugment(std::string name);

protected:
   void Update() override{};
};

/**
 * @brief Class for tracking the information from a branch of the FairRoot source tree.
 *
 * Templated class, for an AtRawEvent it would be instantiated like AtTabInfo<AtRawEvent>.
 * Switched to template to make adding additional info types stored on the TTree trivial to add.
 * @ingroup TabData
 */
template <typename T>
class AtTabInfoFairRoot : public AtTabInfoBase, public DataHandling::Observer {

protected:
   DataHandling::AtBranch &fBranchName;
   T *fInfo{nullptr};

public:
   AtTabInfoFairRoot(DataHandling::AtBranch &branch) : AtTabInfoBase(), fBranchName(branch)
   {
      fBranchName.Attach(this);
   }
   ~AtTabInfoFairRoot() { fBranchName.Detach(this); }

   void Init() override {}
   std::string GetDefaultName() override { return T::Class_Name(); }

   T *GetInfo() { return fInfo; }

   void Update(DataHandling::Subject *changedSubject) override
   {
      if (changedSubject == &fBranchName)
         Update();
   }

protected:
   void Update() override
   {
      auto branchName = fBranchName.GetBranchName();
      auto array = dynamic_cast<TClonesArray *>(FairRootManager::Instance()->GetObject(branchName));
      if (array != nullptr)
         fInfo = dynamic_cast<T *>(array->At(0));
   }
};

#endif
