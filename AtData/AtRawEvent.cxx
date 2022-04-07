/*********************************************************************
 *   AtTPC AtRawEvent Stores a RawEvent composed by the AtTC pads    *
 *   Author: Y. Ayyad            				     *
 *   Log: 07-03-2015 17:16 JST					     *
 *   Adapted from STRawEvent from SPiRITROOT by G. Jhang             *
 *   Edited by Adam Anthony 2/17/2020                                *
 *********************************************************************/

#include <FairLogger.h>
#include <iostream>

#include "AtRawEvent.h"
#include "AtPad.h"

ClassImp(AtRawEvent);

AtRawEvent::AtRawEvent() : TNamed("AtRawEvent", "Raw event container")
{
   SetNumberOfTimestamps(1);
}

AtRawEvent::AtRawEvent(const AtRawEvent &obj)
{
   std::cout << "Copying AtRawEvent " << obj.fEventID << " with " << obj.fPadList.size() << " pads" << std::endl;
   fEventID = obj.fEventID;
   fAuxPadMap = obj.fAuxPadMap;
   for (const auto &pad : obj.fPadList)
      fPadList.push_back(std::make_unique<AtPad>(*pad));

   fTimestamp = obj.fTimestamp;
   fIsGood = obj.fIsGood;
   fIsInGate = obj.fIsInGate;
   fSimMCPointMap = obj.fSimMCPointMap;
}

void AtRawEvent::Clear(Option_t *opt)
{
   fEventID = 0;
   fPadList.clear();
   fAuxPadMap.clear();
   fTimestamp.clear();

   fIsGood = kTRUE;
}

std::pair<AtAuxPad *, bool> AtRawEvent::AddAuxPad(std::string auxName)
{
   auto ret = fAuxPadMap.emplace(auxName, AtAuxPad(auxName));
   auto pad = &(ret.first->second);
   return std::pair<AtAuxPad *, bool>(pad, ret.second);
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

AtPad *AtRawEvent::GetPad(Int_t padNum)
{
   for (auto &pad : fPadList)
      if (pad->GetPadNum() == padNum)
         return pad.get();
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
