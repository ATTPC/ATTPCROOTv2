#include "AtFilterFPN.h"

#include "AtPadReference.h" // for AtPadReference
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <memory>        // for allocator
#include <unordered_map> // for _Node_const_iterator, operator!=, unorde...

AtFilterFPN::AtFilterFPN(AtMapPtr map, bool avgAgets, Int_t num)
   : AtFilterSubtraction(map, num, avgAgets ? 4 : 16), fAverageAgets(avgAgets)
{
}

void AtFilterFPN::InitEvent(AtRawEvent *event)
{
   Clear();

   for (const auto &[ref, pad] : event->GetFpnPads())
      processPad(ref, pad);
   AverageBaseline();
   fEventNumber = event->GetEventID();
}

bool AtFilterFPN::isValidPad(const AtPad &pad)
{
   for (int i = 2; i < 510; ++i)
      if (std::abs(pad.GetADC(i) - pad.GetADC(i - 1)) > fThreshold) {
         LOG(debug) << "Skipping " << std::abs(pad.GetADC(i) - pad.GetADC(i - 1)) << " at " << i;
         return false;
      }
   LOG(debug) << "Adding pad";
   return true;
}

int AtFilterFPN::getAsad(const AtPadReference &ref)
{
   if (fAverageAgets)
      return ref.asad;
   return 4 * ref.asad + ref.aget;
}
