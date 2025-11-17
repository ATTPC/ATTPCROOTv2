#include "AtGaggEvent.h"

#include "AtContainerManip.h"

#include <Rtypes.h>

#include <algorithm>
#include <string> // for string

ClassImp(AtGaggEvent);

AtGaggEvent::AtGaggEvent() : AtBaseEvent("AtGaggEvent") {}

AtGaggEvent::AtGaggEvent(const AtGaggEvent &copy) : AtBaseEvent(copy),
                                              fMultiplicity1(copy.fMultiplicity1), fMultiplicity2(copy.fMultiplicity2)
{
   for (int i = 0; i < 25; i++) {
      fE1[i] = copy.fE1[i];
      fADCMax1[i] = copy.fADCMax1[i];
   }
   for (int i = 0; i < 16; i++) {
      fE2[i] = copy.fE2[i];
      fADCMax2[i] = copy.fADCMax2[i];
   }
}

AtGaggEvent &AtGaggEvent::operator=(AtGaggEvent object)
{
   swap(*this, object);
   return *this;
}

void AtGaggEvent::Clear(Option_t *opt)
{
   AtBaseEvent::Clear(opt);
   for (int i = 0; i < 25; i++) {
      fE1[i] = -1;
      fADCMax1[i] = -1;
   }
   for (int i = 0; i < 16; i++) {
      fE2[i] = -1;
      fADCMax2[i] = -1;
   }

   fMultiplicity1 = 0;
   fMultiplicity2 = 0;
}
