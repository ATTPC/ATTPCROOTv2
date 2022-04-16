#ifndef AtTRACKFINDERHCTASK_H
#define AtTRACKFINDERHCTASK_H

#include <vector>

// ROOT
#include <TClonesArray.h>

// AtTPCROOT classes
#include "AtDigiPar.h"
#include "AtEvent.h"
#include "AtHit.h"
#include "AtProtoEvent.h"
#include "AtTrackFinderHC.h"

// FAIRROOT classes
#include <FairLogger.h>
#include <FairTask.h>

class AtTrackFinderHCTask : public FairTask {
public:
   AtTrackFinderHCTask();
   ~AtTrackFinderHCTask();

   virtual InitStatus Init();
   virtual void Exec(Option_t *option);
   virtual void SetParContainers();
   virtual void Finish();

   void SetPersistence(Bool_t value = kTRUE);

private:
   TClonesArray *fEventHArray;
   TClonesArray *fTrackFinderHCArray;

   FairLogger *fLogger;
   AtDigiPar *fPar;

   // AtEvent *fEvent;

   Bool_t kIsPersistence;

   ClassDef(AtTrackFinderHCTask, 1);
};

#endif
