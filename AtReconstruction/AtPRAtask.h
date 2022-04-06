#ifndef AtPRATASK_H
#define AtPRATASK_H

#include <vector>

// ROOT
#include <TClonesArray.h>

// AtTPCROOT classes
#include "AtEvent.h"
#include "AtPatternEvent.h"
#include "AtDigiPar.h"
#include "AtHit.h"
#include "AtPRA.h"
#include "AtTrackFinderHC.h"

// FAIRROOT classes
#include <FairTask.h>
#include <FairLogger.h>

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

   void SetScluster(float s) { fHCs = s; }
   void SetKtriplet(size_t k) { fHCk = k; }
   void SetNtriplet(size_t n) { fHCn = n; }
   void SetMcluster(size_t m) { fHCm = m; }
   void SetRsmooth(float r) { fHCr = r; }
   void SetAtriplet(float a) { fHCa = a; }
   void SetTcluster(float t) { fHCt = t; }
   void SetPadding(size_t padding) { fHCpadding = padding; }

   void SetMaxNumHits(Int_t maxHits) { fMaxNumHits = maxHits; }
   void SetMinNumHits(Int_t minHits) { fMinNumHits = minHits; }

   void SetPrunning() { kSetPrunning = kTRUE; }
   void SetkNN(Double_t knn) { fKNN = knn; }
   void SetStdDevMulkNN(Double_t stdDevMul) { fStdDevMulkNN = stdDevMul; }
   void SetkNNDist(Double_t dist) { fkNNDist = dist; }

private:
   TClonesArray *fEventHArray;
   TClonesArray *fPatternEventArray;

   FairLogger *fLogger;
   AtDigiPar *fPar;

   AtPATTERN::AtPRA *fPRA;

   Int_t fPRAlgorithm;

   Int_t fMinNumHits;
   Int_t fMaxNumHits;

   Bool_t kIsPersistence;

   // HC parameters
   float fHCs;
   size_t fHCk;
   size_t fHCn;
   size_t fHCm;
   float fHCr;
   float fHCa;
   float fHCt;
   size_t fHCpadding;

   // Prunning parameters
   Bool_t kSetPrunning;
   Int_t fKNN;             //<! Number of nearest neighbors kNN
   Double_t fStdDevMulkNN; //<! Std dev multiplier for kNN
   Double_t fkNNDist;      //<! Distance threshold for outlier rejection in kNN

   ClassDef(AtPRAtask, 1);
};

#endif
