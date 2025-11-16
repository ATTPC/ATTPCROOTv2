#ifndef AtSITASK_H
#define AtSITASK_H

#include "AtPSA.h"

#include <FairTask.h>

#include <Rtypes.h> // for THashConsistencyHolder, Bool_t, ClassDef, Opti...
#include <TClonesArray.h>
#include <TString.h>

#include <memory>
class TBuffer;
class TClass;
class TMemberInspector;

class AtSiTask : public FairTask {
private:
   TString fInputBranchName;
   TString fOutputBranchName;

   TClonesArray *fRawEventArray{nullptr};
   TClonesArray fSiEventArray;

   std::unique_ptr<AtPSA> fPSA;

   Bool_t fIsPersistence{false};

public:
   AtSiTask(std::unique_ptr<AtPSA> psaMethod);
   ~AtSiTask() = default;

   void SetPersistence(Bool_t value);
   void SetInputBranch(TString branchName);
   void SetOutputBranch(TString branchName);
   virtual InitStatus Init();
   virtual void Exec(Option_t *opt);
};

#endif
