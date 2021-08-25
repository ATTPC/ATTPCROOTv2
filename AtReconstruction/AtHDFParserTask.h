#ifndef _AtHDFPARSERTASK_H_
#define _AtHDFPARSERTASK_H_

#include "FairTask.h"
#include "FairLogger.h"

#include "AtHDFParser.h"
#include "AtTpcProtoMap.h"
#include "AtPedestal.h"
#include "AtRawEvent.h"
#include "AtMap.h"
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

   bool SetAuxChannel(PadReference pad, std::string channel_name);

   std::pair<bool, std::string> FindAuxChannel(uint32_t hash);
   Bool_t SetBaseLineSubtraction(Bool_t value = kFALSE);

   void SetNumberTimestamps(int numTimestamps) { fNumberTimestamps = numTimestamps; };
   int GetNumberTimestamps() { return fNumberTimestamps; };
   std::size_t GetNumEvents() { return fNumEvents; };

   AtMap *GetMap() { return fAtMapPtr; }

   virtual InitStatus Init();
   virtual void SetParContainers();
   virtual void Exec(Option_t *opt);
   virtual void FinishEvent();

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
   std::unordered_map<PadReference, std::string> fAuxTable;

   ClassDef(AtHDFParserTask, 2);
};

#endif
