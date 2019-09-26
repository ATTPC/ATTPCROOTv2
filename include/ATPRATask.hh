#ifndef ATPRATASK_H
#define ATPRATASK_H

#include <vector>

//ROOT
#include "TClonesArray.h"

// ATTPCROOT classes
#include "ATEvent.hh"
#include "ATPatternEvent.hh"
#include "ATDigiPar.hh"
#include "ATHit.hh"
#include "ATPRA.hh"
#include "ATTrackFinderHC.hh"

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"

class ATPRATask : public FairTask
{
public:
	ATPRATask();
	~ATPRATask();

	virtual InitStatus Init();
	virtual void Exec(Option_t* option);
	virtual void SetParContainers();
	virtual void Finish();

  void SetPersistence(Bool_t value = kTRUE);
  void SetPRAlgorithm(Int_t value = 0);

private:

	TClonesArray *fEventHArray;
	TClonesArray *fPatternEventArray;

	FairLogger *fLogger;
	ATDigiPar *fPar;

	ATPATTERN::ATPRA *fPRA;

    Int_t fPRAlgorithm;
    Int_t fMinNumHits;

	Bool_t kIsPersistence;


  ClassDef(ATPRATask, 1);

};

  #endif
