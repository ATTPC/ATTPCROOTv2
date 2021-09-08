/*********************************************************************
 *   AtTPC AtRawEvent Stores a RawEvent composed by the AtTC pads    *
 *   Author: Y. Ayyad            				     *
 *   Log: 07-03-2015 17:16 JST					     *
 *   Adapted from STRawEvent from SPiRITROOT by G. Jhang             *
 *   Edited by Adam Anthony 2/17/2020                                *
 *********************************************************************/

#include <iostream>
#include <iomanip>

#include "AtRawEvent.h"
#include "AtPad.h"

ClassImp(AtRawEvent);

AtRawEvent::AtRawEvent() : TNamed("AtRawEvent", "Raw event container")
{
   fEventID = -1;
   SetNumberOfTimestamps(1);
   fIsGood = kTRUE;
   fIsinGate = kFALSE;

   fSimMCPointMap.clear();
}

AtRawEvent::AtRawEvent(AtRawEvent *object) : TNamed("AtRawEvent", "Raw event container")
{
   fEventID = object->GetEventID();
   fPadList = object->GetPads();
   fAuxPadMap = object->GetAuxPads();
   fIsGood = object->IsGood();

   fSimMCPointMap = object->GetSimMCPointMap();
   fTimestamp = *(object->GetTimestamps());
}

void AtRawEvent::Clear()
{
   fEventID = 0;
   fPadList.clear();
   fAuxPadMap.clear();

   fIsGood = kTRUE;
}

// setters
void AtRawEvent::SetEventID(ULong_t evtid)
{
   fEventID = evtid;
}
AtPad &AtRawEvent::AddPad(int padNum)
{
   fPadList.emplace_back(AtPad{padNum});
   return fPadList.back();
}
std::pair<AuxPadMap::iterator, bool> AtRawEvent::AddAuxPad(std::string auxName)
{
   auto ret = fAuxPadMap.emplace(auxName, AtPad{});
   auto &pad = ret.first->second;
   pad.SetIsAux(true);
   pad.SetAuxName(auxName);
   return ret;
}
void AtRawEvent::SetIsGood(Bool_t value)
{
   fIsGood = value;
}
void AtRawEvent::SetTimestamp(ULong64_t timestamp, int index)
{
   fTimestamp.at(index) = timestamp;
}
void AtRawEvent::SetNumberOfTimestamps(int numTS)
{
   fTimestamp.resize(numTS, 0);
}
void AtRawEvent::SetIsExtGate(Bool_t value)
{
   fIsinGate = value;
}
void AtRawEvent::SetSimMCPointMap(std::multimap<Int_t, std::size_t> map)
{
   fSimMCPointMap = map;
};

void AtRawEvent::RemovePad(Int_t padNum)
{
   for (auto it = fPadList.begin(); it != fPadList.end(); ++it)
      if (it->GetPadNum() == padNum)
         fPadList.erase(it);
}

// getters
ULong_t AtRawEvent::GetEventID() const
{
   return fEventID;
}
Int_t AtRawEvent::GetNumPads() const
{
   return fPadList.size();
}
Int_t AtRawEvent::GetNumAuxPads() const
{
   return fAuxPadMap.size();
}
Bool_t AtRawEvent::IsGood() const
{
   return fIsGood;
}
ULong64_t AtRawEvent::GetTimestamp(int index) const
{
   return fTimestamp.at(index);
}
const std::vector<ULong64_t> *AtRawEvent::GetTimestamps() const
{
   return &fTimestamp;
}
PadVector &AtRawEvent::GetPads()
{
   return fPadList;
}
AuxPadMap &AtRawEvent::GetAuxPads()
{
   return fAuxPadMap;
}
Bool_t AtRawEvent::GetIsExtGate() const
{
   return fIsinGate;
}
std::multimap<Int_t, std::size_t> &AtRawEvent::GetSimMCPointMap()
{
   return fSimMCPointMap;
}

AtPad *AtRawEvent::GetPad(Int_t padNum)
{
   for (auto &pad : fPadList)
      if (pad.GetPadNum() == padNum)
         return &pad;
   return nullptr;
}

AtPad *AtRawEvent::GetAuxPad(std::string auxName)
{
   auto padIt = fAuxPadMap.find(auxName);
   if (padIt == fAuxPadMap.end())
      return nullptr;
   else
      return &(padIt->second);
}
