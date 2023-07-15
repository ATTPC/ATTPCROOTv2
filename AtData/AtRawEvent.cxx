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

ClassImp(AtRawEvent);

AtRawEvent::AtRawEvent(const AtRawEvent &obj)
   : AtBaseEvent(obj), fFpnMap(obj.fFpnMap), fSimMCPointMap(obj.fSimMCPointMap)
{
   for (const auto &pad : obj.fPadList)
      fPadList.push_back(pad->ClonePad());
}

AtRawEvent &AtRawEvent::operator=(AtRawEvent object)
{
   swap(*this, object);
   return *this;
}

/*
void AtRawEvent::CopyAllButData(const AtRawEvent *event)
{
   fEventID = event->fEventID;
   fTimestamp = event->fTimestamp;
   fIsGood = event->fIsGood;
   fIsInGate = event->fIsInGate;
}
*/

void AtRawEvent::Clear(Option_t *opt)
{
   AtBaseEvent::Clear(opt);

   fPadList.clear();
   fFpnMap.clear();
   fSimMCPointMap.clear();
   fGTraceList.clear();
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

/**
 * @brief Create a new FPN channel.
 * @return Pointer to newly created pad
 */
AtPad *AtRawEvent::AddFPN(const AtPadReference &ref)
{
   auto [it, added] = fFpnMap.emplace(ref, AtPad());
   return &(it->second);
}
