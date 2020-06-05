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
#include <unordered_map>
#include <sstream>


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
  Bool_t SetInitialEvent(std::size_t inievent);
  Bool_t SetOldFormat(Bool_t oldF = kFALSE);
  bool   SetAuxChannel(uint32_t hash,std::string channel_name);
  std::pair<bool,std::string> FindAuxChannel(uint32_t hash);

  void SetTimestampIndex(int index) { fTimestampIndex = index; };
  int  GetTimestampIndex()    { return fTimestampIndex; };
  std::size_t GetNumEvents() { return fNumEvents; };
  
  virtual InitStatus Init();
  virtual void SetParContainers();
  virtual void Exec(Option_t *opt);
  virtual void FinishEvent();

  static uint32_t CalculateHash(uint8_t cobo, uint8_t asad, uint8_t aget, uint8_t channel)
    {
        auto wcobo    = uint32_t(cobo);
        auto wasad    = uint32_t(asad);
        auto waget    = uint32_t(aget);
        auto wchannel = uint32_t(channel);

        auto result = wchannel + waget*100 + wasad*10000 + wcobo*1000000;

        return result;
    }
  
private:
  
  FairLogger *fLogger;
  ATHDFParser* HDFParser;
  ATDigiPar *fPar;
  TClonesArray *fRawEventArray;       
  ATRawEvent* fRawEvent;
  
  
  std::size_t    fIniEventID;                  
  std::size_t    fNumEvents; 
  std::size_t    fEventID;         
  
  Bool_t fIsPersistence;  
  std::string fFileName;
  Bool_t fIsOldFormat; 

  char const *fMap;
  AtTpcMap *fAtMapPtr;

  typedef boost::multi_array<double,3> multiarray;
  typedef multiarray::index index;
  multiarray AtPadCoordArr;

  Int_t  kOpt;
  Int_t  fTimestampIndex;
  Bool_t fIsProtoGeoSet;
  Bool_t fIsProtoMapSet;

  std::vector<std::string> fEventsByName;
  std::unordered_map<uint32_t,std::string>            fAuxTable;
  std::unordered_map<uint32_t,std::string>::iterator  fAuxTableIt;
             
  
  ClassDef(ATHDFParserTask, 1);
};

#endif
