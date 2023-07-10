#ifndef _ATUNPACKER_H_
#define _ATUNPACKER_H_

#include <Rtypes.h>
#include <TObject.h>

#include <memory>
#include <string>
#include <utility>

class AtRawEvent;
class AtMap;
class TBuffer;
class TClass;
class TMemberInspector;

using mapPtr = std::shared_ptr<AtMap>;

class AtUnpacker : public TObject {
protected:
   mapPtr fMap;
   std::string fInputFileName;
   Long64_t fEventID = 0;     // Internal event ID to be unpacked next (stored in AtRawEvent)
   Long64_t fDataEventID = 0; // Event ID as tracked whatever produced the data being unpacked
   AtRawEvent *fRawEvent{};

   Bool_t fSaveFPN{false};

public:
   AtUnpacker(mapPtr map);
   ~AtUnpacker() = default;

   virtual void SetInputFileName(std::string fileName) { fInputFileName = std::move(fileName); }
   void SetMap(mapPtr map) { fMap = map; }
   void SetInitialEventID(Long64_t evtID) { fEventID = evtID; }
   virtual void SetSaveFPN(bool val = true) { fSaveFPN = val; }
   Long64_t GetNextEventID() { return fEventID; }

   virtual void Init() = 0;
   // Pass by ref to ensure it's a valid object
   virtual void FillRawEvent(AtRawEvent &event) = 0;
   // Returns true if we have unpacked the last valid event
   virtual bool IsLastEvent() = 0;
   virtual Long64_t GetNumEvents() = 0;

   ClassDef(AtUnpacker, 1)
};

#endif //#ifndef _ATUNPACKER_H_
