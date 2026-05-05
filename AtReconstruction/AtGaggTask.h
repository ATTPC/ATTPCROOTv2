#ifndef AtGAGGTASK_H
#define AtGAGGTASK_H

#include "AtGAGGMap.h"
#include "AtPSASi.h"

#include <FairTask.h>

#include <Rtypes.h> // for THashConsistencyHolder, Bool_t, ClassDef, Opti...
#include <TClonesArray.h>
#include <TString.h>

#include <memory>
class TBuffer;
class TClass;
class TMemberInspector;

class AtGaggTask : public FairTask {
private:
   TString fInputBranchName;
   TString fOutputBranchName;

   TClonesArray *fRawEventArray{nullptr};
   TClonesArray fGaggEventArray;

   std::unique_ptr<AtPSASi> fPSA;
   std::unique_ptr<AtGAGGMap> fGaggMap;

   Bool_t fIsPersistence{false};

public:
   AtGaggTask(std::unique_ptr<AtPSASi> psaMethod);
   AtGaggTask(std::unique_ptr<AtPSASi> psaMethod, std::unique_ptr<AtGAGGMap>);
   ~AtGaggTask() = default;

   void SetPersistence(Bool_t value);
   void SetInputBranch(TString branchName);
   void SetOutputBranch(TString branchName);
   virtual InitStatus Init();
   virtual void Exec(Option_t *opt);
};

#endif
