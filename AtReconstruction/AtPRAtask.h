#ifndef AtPRATASK_H
#define AtPRATASK_H

#include <vector>

// ROOT
#include "TClonesArray.h"

// AtTPCROOT classes
#include "AtEvent.h"
#include "AtPatternEvent.h"
#include "AtDigiPar.h"
#include "AtHit.h"
#include "AtPRA.h"
#include "AtTrackFinderHC.h"

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

class AtPRAtask : public FairTask {
public:
   AtPRAtask();
   ~AtPRAtask();

   virtual InitStatus Init();
   virtual void Exec(Option_t *option);
   virtual void SetParContainers();
   virtual void Finish();

   void SetPersistence(Bool_t value = kTRUE);
   void SetPRAlgorithm(Int_t value = 0);

   void SetScluster(float s){ fHCs = s;}
   void SetKtriplet(size_t k){ fHCk = k;}
   void SetNtriplet(size_t n){  fHCn = n;}
   void SetMcluster(size_t m){  fHCm = m;}
   void SetRsmooth(float r){  fHCr = r;}
   void SetAtriplet(float a){ fHCa = a;}
   void SetTcluster(float t){ fHCt = t;}
   void SetPadding(size_t padding){  fHCpadding = padding;}        
  
private:
   TClonesArray *fEventHArray;
   TClonesArray *fPatternEventArray;

   FairLogger *fLogger;
   AtDigiPar *fPar;

   AtPATTERN::AtPRA *fPRA;

   Int_t fPRAlgorithm;
   Int_t fMinNumHits;

   Bool_t kIsPersistence;

   //HC parameters
  float fHCs;
  size_t fHCk;
  size_t fHCn;
  size_t fHCm;
  float fHCr;
  float fHCa;
  float fHCt;
  size_t fHCpadding;

   ClassDef(AtPRAtask, 1);
};

#endif
