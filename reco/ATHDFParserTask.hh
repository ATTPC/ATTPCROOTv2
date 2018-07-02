#ifndef _ATHDFPARSERTASK_H_
#define _ATHDFPARSERTASK_H_

#include "FairTask.h"
#include "FairLogger.h"

#include "ATHDFParser.hh"
#include "AtTpcMap.h"
#include "ATPedestal.hh"
#include "ATRawEvent.hh"

#include "ATDigiPar.hh"

// ROOT classes
#include "TClonesArray.h"
#include "TString.h"

// STL
#include <vector>
#include <memory>

class ATHDFParserTask : public FairTask {
  
public:
  
  ATHDFParserTask();
  ~ATHDFParserTask();
  
  void SetPersistence(Bool_t value = kTRUE); 
  Int_t ReadEvent(Int_t eventID);
  void SetFileName(std::string filename) {fFileName = filename;}
  
  virtual InitStatus Init();
  virtual void SetParContainers();
  virtual void Exec(Option_t *opt);
  virtual void FinishEvent();
  
private:
  
  FairLogger *fLogger;
  ATHDFParser* HDFParser;
  ATDigiPar *fPar;
  TClonesArray *fRawEventArray;       
  ATRawEvent* fRawEvent;
  
  Long64_t    fEventID;                  
  std::size_t fNumEvents;            
  
  Bool_t fIsPersistence;  
  std::string fFileName;            
  
  ClassDef(ATHDFParserTask, 1);
};

#endif
