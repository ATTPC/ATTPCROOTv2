#ifndef AtPSAtASK_H
#define AtPSAtASK_H

#include <Rtypes.h> // for THashConsistencyHolder, Bool_t, ClassDef, Opti...
class TBuffer;
class TClass;
class TMemberInspector;

// FAIRROOT classes
#include <FairTask.h>

// AtTPCROOT classes
class AtPSA;

// ROOT classes
#include <TString.h>
class TClonesArray;

class AtPSAtask : public FairTask {
private:
   TClonesArray *fRawEventArray{};
   TClonesArray *fEventHArray;
   TClonesArray *fMCPointArray{};

   TString fInputBranchName;
   TString fOutputBranchName;
   TString fSimulatedPointBranchName;

   AtPSA *fPSA;

   Bool_t fIsPersistence;

public:
   AtPSAtask(AtPSA *psaMethod);
   ~AtPSAtask();

   void SetPersistence(Bool_t value);
   void SetInputBranch(TString branchName);
   void SetOutputBranch(TString branchName);
   void SetSimlulatedPointBranch(TString branchName);
   virtual InitStatus Init();
   virtual void Exec(Option_t *opt);

   ClassDef(AtPSAtask, 2);
};

#endif
