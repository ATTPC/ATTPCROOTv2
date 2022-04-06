/*********************************************************************
 *   AtTPC AtRawEvent Stores a RawEvent composed by the AtTC pads    *
 *   Author: Y. Ayyad            				     *
 *   Log: 07-03-2015 17:16 JST					     *
 *   Adapted from STRawEvent from SPiRITROOT by G. Jhang             *
 *   Edited by Adam Anthony 2/17/2020                                *
 *                                                                   *
 *********************************************************************/

#ifndef AtRAWEVENT_H
#define AtRAWEVENT_H

#include <Rtypes.h>
#include <TNamed.h>
#include <vector>
#include <map>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "AtAuxPad.h"

class AtPad;
class TBuffer;
class TClass;
class TMemberInspector;

using AuxPadMap = std::map<std::string, AtAuxPad>;
using AtPadPtr = std::unique_ptr<AtPad>;
using PadVector = std::vector<AtPadPtr>;

class AtRawEvent : public TNamed {
private:
   ULong_t fEventID = -1;
   PadVector fPadList;
   AuxPadMap fAuxPadMap;
   std::vector<ULong64_t> fTimestamp;

   Bool_t fIsGood = true;
   Bool_t fIsInGate = false;

   std::multimap<Int_t, std::size_t> fSimMCPointMap; //<! Monte Carlo Point - Hit map for kinematics

   friend class AtFilterTask;

public:
   AtRawEvent();
   AtRawEvent(AtRawEvent &&obj) = default;
   AtRawEvent &operator=(AtRawEvent &&obj) = default;
   AtRawEvent(const AtRawEvent &object);
   ~AtRawEvent() = default;

   void Clear(Option_t *opt = nullptr) override;

   // As an input takes parameters for any constructor of AtPad
   template <typename... Ts>
   AtPad *AddPad(Ts &&...params)
   {
      fPadList.push_back(std::make_unique<AtPad>(std::forward<Ts>(params)...));
      return fPadList.back().get();
   }
   // Returns a pointer to the newly added pad, or existing pad if auxName is already used
   // bool returned is true if insert occurred.
   std::pair<AtAuxPad *, bool> AddAuxPad(std::string auxName);
   AtPad *GetPad(Int_t padNum);
   AtPad *GetAuxPad(std::string auxPad);

   void RemovePad(Int_t padNum);
   void SetSimMCPointMap(std::multimap<Int_t, std::size_t> map) { fSimMCPointMap = std::move(map); }

   // setters
   void SetEventID(ULong_t evtid) { fEventID = evtid; }
   void SetIsGood(Bool_t value) { fIsGood = value; }
   void SetTimestamp(ULong64_t timestamp, int index = 0);
   void SetNumberOfTimestamps(int numTS) { fTimestamp.resize(numTS, 0); }
   void SetIsExtGate(Bool_t value) { fIsInGate = value; }

   // getters
   ULong_t GetEventID() const { return fEventID; }
   Int_t GetNumPads() const { return fPadList.size(); }
   Int_t GetNumAuxPads() const { return fAuxPadMap.size(); }
   ULong64_t GetTimestamp(int index = 0) const { return index < fTimestamp.size() ? fTimestamp.at(index) : 0; }
   const std::vector<ULong64_t> &GetTimestamps() const { return fTimestamp; }
   Bool_t IsGood() const { return fIsGood; }
   Bool_t GetIsExtGate() const { return fIsInGate; }

   const PadVector &GetPads() { return fPadList; }
   const AuxPadMap &GetAuxPads() { return fAuxPadMap; }
   std::multimap<Int_t, std::size_t> &GetSimMCPointMap() { return fSimMCPointMap; }

   ClassDefOverride(AtRawEvent, 5);
};

#endif
