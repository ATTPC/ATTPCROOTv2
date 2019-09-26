#ifndef ATTRACKFINDERHCTASK_H
#define ATTRACKFINDERHCTASK_H

#include <vector>

//ROOT
#include "TClonesArray.h"

// ATTPCROOT classes
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATDigiPar.hh"
#include "ATHit.hh"
#include "ATTrackFinderHC.hh"

// FAIRROOT classes
#include "FairTask.h"
#include "FairLogger.h"



class ATTrackFinderHCTask : public FairTask
{
public:
	ATTrackFinderHCTask();
	~ATTrackFinderHCTask();

	virtual InitStatus Init();
	virtual void Exec(Option_t* option);
	virtual void SetParContainers();
	virtual void Finish();

  void SetPersistence(Bool_t value = kTRUE);

private:

	TClonesArray *fEventHArray;
	TClonesArray *fTrackFinderHCArray;

	FairLogger *fLogger;
	ATDigiPar *fPar;

	//ATEvent *fEvent;

	Bool_t kIsPersistence;


  ClassDef(ATTrackFinderHCTask, 1);

};

  #endif
