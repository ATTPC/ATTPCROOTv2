#ifndef ATBASEEVENT_H
#define ATBASEEVENT_H
#include "AtAuxPad.h"

#include <Rtypes.h>
#include <TNamed.h>

#include <algorithm> // for max
#include <map>       // for swap, map
#include <string>    // for string, allocator
#include <utility>   // for swap, move, pair
#include <vector>    // for vector, swap
class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief Base class for all event types in ATTPCROOT
 */
class AtBaseEvent : public TNamed {

protected:
   using AuxPadMap = std::map<std::string, AtAuxPad>;
   ULong_t fEventID = -1;
   Bool_t fIsGood = true;
   Bool_t fIsInGate = false;

   std::vector<ULong64_t> fTimestamp{1};
   AuxPadMap fAuxPadMap;
   std::string fEventName;

public:
   AtBaseEvent(std::string name = "AtBaseEvent");
   virtual ~AtBaseEvent() = default;
   AtBaseEvent(AtBaseEvent &&) = default;
   AtBaseEvent(const AtBaseEvent &) = default;
   AtBaseEvent &operator=(AtBaseEvent object);

   friend void swap(AtBaseEvent &first, AtBaseEvent &second)
   {
      using std::swap;
      swap(first.fEventID, second.fEventID);
      swap(first.fIsGood, second.fIsGood);
      swap(first.fIsInGate, second.fIsInGate);
      swap(first.fTimestamp, second.fTimestamp);
      swap(first.fAuxPadMap, second.fAuxPadMap);
      swap(first.fEventName, second.fEventName);
   };

   void Clear(Option_t *opt) override;

   /**
    * @brief Add new auxilary pad (AtAuxPad) to event
    * @param Name of new auxiliary pad
    * @return Returns a pointer to the newly added pad, or existing pad if auxName is already used,
    * bool returned is true if insert occurred.
    */
   std::pair<AtAuxPad *, bool> AddAuxPad(std::string auxName);

   void SetEventID(ULong_t evtid) { fEventID = evtid; }
   void SetIsGood(Bool_t value) { fIsGood = value; }
   void SetTimestamp(ULong64_t timestamp, int index = 0);
   void SetNumberOfTimestamps(int numTS) { fTimestamp.resize(numTS, 0); }
   void SetIsExtGate(Bool_t value) { fIsInGate = value; }
   void SetEventName(std::string name) { fEventName = name; }

   ULong_t GetEventID() const { return fEventID; }
   ULong64_t GetTimestamp(int index = 0) const { return index < fTimestamp.size() ? fTimestamp.at(index) : 0; }
   const std::vector<ULong64_t> &GetTimestamps() const { return fTimestamp; }
   Bool_t IsGood() const { return fIsGood; }
   Bool_t GetIsExtGate() const { return fIsInGate; }
   std::string GetEventName() const { return fEventName; }

   AtAuxPad *GetAuxPad(std::string auxPad)
   {
      return const_cast<AtAuxPad *>(const_cast<const AtBaseEvent *>(this)->GetAuxPad(std::move(auxPad)));
   }
   const AtAuxPad *GetAuxPad(std::string auxPad) const;
   const AuxPadMap &GetAuxPads() const { return fAuxPadMap; }

   ClassDefOverride(AtBaseEvent, 1)
};

#endif //#ifndef ATBASEEVENT_H
