/*********************************************************************
 *   AtTPC AtRawEvent Stores a RawEvent composed by the AtTC pads    *
 *   Author: Y. Ayyad            				     *
 *   Log: 07-03-2015 17:16 JST					     *
 *   Adapted from STRawEvent from SPiRITROOT by G. Jhang             *
 *   Edited by Adam Anthony 2/17/2020                                *
 *********************************************************************/

#include "AtRawEvent.h"

#include "AtPad.h"
#include "AtPadReference.h" // for AtPadReference (ptr only), operator==

#include <FairLogger.h>

ClassImp(AtRawEvent);

AtRawEvent::AtRawEvent() : TNamed("AtRawEvent", "Raw event container")
{
   SetNumberOfTimestamps(1);
}

AtRawEvent::AtRawEvent(const AtRawEvent &obj)
   : fEventID(obj.fEventID), fAuxPadMap(obj.fAuxPadMap), fIsInGate(obj.fIsInGate), fSimMCPointMap(obj.fSimMCPointMap),
     fIsGood(obj.fIsGood), fFpnMap(obj.fFpnMap)
{
   for (const auto &pad : obj.fPadList)
      fPadList.push_back(pad->ClonePad());
}

AtRawEvent &AtRawEvent::operator=(AtRawEvent object)
{
   swap(*this, object);
   return *this;
}

void AtRawEvent::CopyAllButData(const AtRawEvent *event)
{
   fEventID = event->fEventID;
   fTimestamp = event->fTimestamp;
   fIsGood = event->fIsGood;
   fIsInGate = event->fIsInGate;
}

void AtRawEvent::Clear(Option_t *opt)
{
   fEventID = -1;
   fPadList.clear();
   fAuxPadMap.clear();
   fTimestamp.clear();
   fSimMCPointMap.clear();

   fIsGood = true;
   fIsInGate = false;
}

std::pair<AtAuxPad *, bool> AtRawEvent::AddAuxPad(std::string auxName)
{
   auto ret = fAuxPadMap.emplace(auxName, AtAuxPad(auxName));
   auto pad = &(ret.first->second);
   return {pad, ret.second};
}

void AtRawEvent::SetTimestamp(ULong64_t timestamp, int index)
{
   if (index < fTimestamp.size())
      fTimestamp[index] = timestamp;
   else
      LOG(error) << "Failed to add timestamp with index " << index << " to raw event. Max number of timestamps is "
                 << fTimestamp.size();
}

void AtRawEvent::RemovePad(Int_t padNum)
{
   for (auto it = fPadList.begin(); it != fPadList.end(); ++it)
      if ((*it)->GetPadNum() == padNum)
         fPadList.erase(it);
}

const AtPad *AtRawEvent::GetPad(Int_t padNum) const
{
   for (auto &pad : fPadList)
      if (pad->GetPadNum() == padNum)
         return pad.get();
   return nullptr;
}

const AtPad *AtRawEvent::GetFpn(const AtPadReference &ref) const
{
   auto padIt = fFpnMap.find(ref);
   if (padIt == fFpnMap.end())
      return nullptr;
   else
      return &(padIt->second);
}

const AtAuxPad *AtRawEvent::GetAuxPad(std::string auxName) const
{
   auto padIt = fAuxPadMap.find(auxName);
   if (padIt == fAuxPadMap.end())
      return nullptr;
   else
      return &(padIt->second);
}

/**
 * @brief Create a new FPN channel.
 * @return Pointer to newly created pad
 */
AtPad *AtRawEvent::AddFPN(const AtPadReference &ref)
{
   auto [it, added] = fFpnMap.emplace(ref, AtPad());
   return &(it->second);
}
