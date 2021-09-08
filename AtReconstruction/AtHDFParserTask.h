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

using mapPtr = std::shared_ptr<AtMap>;

class AtHDFParserTask : public FairTask {

private:
   AtHDFParser *HDFParser;
   AtDigiPar *fPar;
   TClonesArray *fRawEventArray;
   AtRawEvent *fRawEvent;
   mapPtr fAtMapPtr;

   std::string fFileName;
   Int_t fNumberTimestamps;
   Bool_t fIsPersistence;
   Bool_t fIsOldFormat;
   Bool_t fIsProtoGeoSet;
   Bool_t fIsProtoMapSet;
   Bool_t fIsBaseLineSubtraction;

   std::size_t fIniEventID;
   std::size_t fNumEvents;
   std::size_t fEventID;

   std::unordered_map<PadReference, std::string> fAuxTable;

   void processHeader();
   void processData();
   void processPad(std::size_t padIndex);
   AtPad &createPadAndSetIsAux(const PadReference &padRef);
   void setDimensions(AtPad &pad);
   Float_t getBaseline(const std::vector<int16_t> &data);
   void setAdc(AtPad &pad, const std::vector<int16_t> &data);

public:
   AtHDFParserTask();
   ~AtHDFParserTask();

   void SetFileName(std::string filename) { fFileName = filename; }
   void SetPersistence(Bool_t value) { fIsPersistence = value; }
   void SetMap(mapPtr map) { fAtMapPtr = map; }
   void SetInitialEvent(std::size_t inievent) { fIniEventID = inievent; }
   void SetOldFormat(Bool_t value) { fIsOldFormat = value; }
   void SetBaseLineSubtraction(Bool_t value) { fIsBaseLineSubtraction = value; }
   void SetNumberTimestamps(int numTimestamps) { fNumberTimestamps = numTimestamps; };
   bool SetAuxChannel(PadReference pad, std::string channel_name);

   int GetNumberTimestamps() { return fNumberTimestamps; };
   std::size_t GetNumEvents() { return fNumEvents; };

   virtual InitStatus Init();
   virtual void SetParContainers();
   virtual void Exec(Option_t *opt);
   virtual void FinishEvent();

   ClassDef(AtHDFParserTask, 3);
};

#endif
