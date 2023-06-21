#include "AtBaseEvent.h"

#include <FairLogger.h>

AtBaseEvent::AtBaseEvent(std::string name) : TNamed(name.c_str(), "") {}

AtBaseEvent &AtBaseEvent::operator=(AtBaseEvent object)
{
   swap(*this, object);
   return *this;
}

void AtBaseEvent::Clear(Option_t *opt)
{
   TNamed::Clear(opt);

   fEventID = -1;
   fAuxPadMap.clear();
   std::fill(fTimestamp.begin(), fTimestamp.end(), 0);

   fIsGood = true;
   fIsInGate = false;
   fEventName = "";
}

const AtAuxPad *AtBaseEvent::GetAuxPad(std::string auxName) const
{
   auto padIt = fAuxPadMap.find(auxName);
   if (padIt == fAuxPadMap.end())
      return nullptr;
   else
      return &(padIt->second);
}

void AtBaseEvent::SetTimestamp(ULong64_t timestamp, int index)
{
   if (index < fTimestamp.size())
      fTimestamp[index] = timestamp;
   else
      LOG(error) << "Failed to add timestamp with index " << index << " to raw event. Max number of timestamps is "
                 << fTimestamp.size();
}

std::pair<AtAuxPad *, bool> AtBaseEvent::AddAuxPad(std::string auxName)
{
   auto ret = fAuxPadMap.emplace(auxName, AtAuxPad(auxName));
   auto pad = &(ret.first->second);
   return {pad, ret.second};
}

ClassImp(AtBaseEvent);
