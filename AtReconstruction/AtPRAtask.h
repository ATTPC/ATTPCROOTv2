#ifndef AtPRAtASK_H
#define AtPRAtASK_H

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

private:
   TClonesArray *fEventHArray;
   TClonesArray *fPatternEventArray;

   FairLogger *fLogger;
   AtDigiPar *fPar;

   AtPAtTERN::AtPRA *fPRA;

   Int_t fPRAlgorithm;
   Int_t fMinNumHits;

   Bool_t kIsPersistence;

   ClassDef(AtPRAtask, 1);
};

#endif
