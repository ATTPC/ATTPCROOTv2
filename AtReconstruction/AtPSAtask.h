#ifndef AtPSAtASK_H
#define AtPSAtASK_H

#include <Rtypes.h> // for THashConsistencyHolder, Bool_t, ClassDef, Opti...
#include <TClonesArray.h>

// FAIRROOT classes
#include <FairTask.h>

class AtPSA;
class TBuffer;
class TClass;
class TMemberInspector;

// ROOT classes
#include <TString.h>
class TClonesArray;

class AtPSAtask : public FairTask {
private:
   TString fInputBranchName;
   TString fOutputBranchName;
   TString fSimulatedPointBranchName;

   TClonesArray *fRawEventArray{nullptr};
   TClonesArray *fMCPointArray{nullptr};
   TClonesArray fEventArray;

   std::unique_ptr<AtPSA> fPSA;

   Bool_t fIsPersistence{false};

public:
   AtPSAtask(std::unique_ptr<AtPSA> psaMethod);
   [[deprecated("Use AtPSAtask(unique_ptr<AtPSA>) instead")]] AtPSAtask(AtPSA *psaMethod);
   ~AtPSAtask() = default;

   void SetPersistence(Bool_t value);
   void SetInputBranch(TString branchName);
   void SetOutputBranch(TString branchName);
   void SetSimlulatedPointBranch(TString branchName);
   virtual InitStatus Init();
   virtual void Exec(Option_t *opt);

   ClassDef(AtPSAtask, 2);
};

#endif
