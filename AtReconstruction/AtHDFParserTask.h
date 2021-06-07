#ifndef _AtHDFPARSERTASK_H_
#define _AtHDFPARSERTASK_H_

#include "FairTask.h"
#include "FairLogger.h"

#include "AtHDFParser.h"
#include "AtMap.h"
#include "AtTpcProtoMap.h"
#include "AtPedestal.h"
#include "AtRawEvent.h"

#include "AtDigiPar.h"

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

class AtHDFParserTask : public FairTask {

public:
   AtHDFParserTask();
   AtHDFParserTask(Int_t opt);
   ~AtHDFParserTask();

   void SetPersistence(Bool_t value = kTRUE);
   void SetFileName(std::string filename) { fFileName = filename; }
   bool SetAtTPCMap(Char_t const *lookup);
   Bool_t SetProtoGeoFile(TString geofile); // Only for Prototype Map
   Bool_t SetProtoMapFile(TString mapfile); // Only for Prototype Map
   Bool_t SetInitialEvent(std::size_t inievent);
   Bool_t SetOldFormat(Bool_t oldF = kFALSE);
   bool SetAuxChannel(uint32_t hash, std::string channel_name);
   std::pair<bool, std::string> FindAuxChannel(uint32_t hash);
   Bool_t SetBaseLineSubtraction(Bool_t value = kFALSE);

   void SetNumberTimestamps(int numTimestamps) { fNumberTimestamps = numTimestamps; };
   int GetNumberTimestamps() { return fNumberTimestamps; };
   std::size_t GetNumEvents() { return fNumEvents; };

   virtual InitStatus Init();
   virtual void SetParContainers();
   virtual void Exec(Option_t *opt);
   virtual void FinishEvent();

   static uint32_t CalculateHash(uint8_t cobo, uint8_t asad, uint8_t aget, uint8_t channel)
   {
      auto wcobo = uint32_t(cobo);
      auto wasad = uint32_t(asad);
      auto waget = uint32_t(aget);
      auto wchannel = uint32_t(channel);

      auto result = wchannel + waget * 100 + wasad * 10000 + wcobo * 1000000;

      return result;
   }

private:
   AtHDFParser *HDFParser;
   AtDigiPar *fPar;
   TClonesArray *fRawEventArray;
   AtRawEvent *fRawEvent;

   std::size_t fIniEventID;
   std::size_t fNumEvents;
   std::size_t fEventID;

   Bool_t fIsPersistence;
   std::string fFileName;
   Bool_t fIsOldFormat;

   char const *fMap;
   AtMap *fAtMapPtr;

   typedef boost::multi_array<double, 3> multiarray;
   typedef multiarray::index index;
   multiarray AtPadCoordArr;

   Int_t kOpt;
   Int_t fNumberTimestamps;
   Bool_t fIsProtoGeoSet;
   Bool_t fIsProtoMapSet;
   Bool_t fIsBaseLineSubtraction;

   std::vector<std::string> fEventsByName;
   std::unordered_map<uint32_t, std::string> fAuxTable;
   std::unordered_map<uint32_t, std::string>::iterator fAuxTableIt;

   ClassDef(AtHDFParserTask, 1);
};

#endif
