#ifndef AtPRATASK_H
#define AtPRATASK_H

#include <FairTask.h> // for FairTask, InitStatus

#include <Rtypes.h>       // for Int_t, Double_t, Bool_t, THashConsistencyH...
#include <TClonesArray.h> // for TClonesArray

#include <stddef.h> // for size_t
class AtDigiPar;
class TBuffer;
class TClass;
class TMemberInspector;
namespace AtPATTERN {
class AtPRA;
}

/**
 * @brief Task for finding patterns in hit clouds.
 *
 * Logic is in class AtPRA and derived types.
 */
class AtPRAtask : public FairTask {
private:
   TClonesArray *fEventHArray{};
   TClonesArray fPatternEventArray;

   AtDigiPar *fPar;

   AtPATTERN::AtPRA *fPRA{};

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
   ClassDef(AtPRAtask, 1);
};

#endif
