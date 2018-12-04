#ifndef _ATHDFPARSERTASK_H_
#define _ATHDFPARSERTASK_H_

#include "FairTask.h"
#include "FairLogger.h"

#include "ATHDFParser.hh"
#include "AtTpcMap.h"
#include "AtTpcProtoMap.h"
#include "ATPedestal.hh"
#include "ATRawEvent.hh"

#include "ATDigiPar.hh"

// ROOT classes
#include "TClonesArray.h"
#include "TString.h"

// STL
#include <vector>
#include <memory>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

class ATHDFParserTask : public FairTask {
  
public:
  
  ATHDFParserTask();
  ATHDFParserTask(Int_t opt);
  ~ATHDFParserTask();
  
  void SetPersistence(Bool_t value = kTRUE); 
  void SetFileName(std::string filename) {fFileName = filename;}
  bool SetATTPCMap(Char_t const *lookup);
  Bool_t SetProtoGeoFile(TString geofile); // Only for Prototype Map
  Bool_t SetProtoMapFile(TString mapfile);  // Only for Prototype Map
  
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

  char const *fMap;
  AtTpcMap *fAtMapPtr;

  typedef boost::multi_array<double,3> multiarray;
  typedef multiarray::index index;
  multiarray AtPadCoordArr;

  Int_t kOpt;
  Bool_t fIsProtoGeoSet;
  Bool_t fIsProtoMapSet;
             
  
  ClassDef(ATHDFParserTask, 1);
};

#endif
