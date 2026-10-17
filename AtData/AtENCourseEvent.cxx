#include "AtENCourseEvent.h"

#include <Rtypes.h>

ClassImp(AtENCourseEvent);

AtENCourseEvent::AtENCourseEvent() : AtBaseEvent("AtENCourseEvent")
{
   fF2PPACs = nullptr;
   fF3PPACs = nullptr;
   for (int i = 0; i < 4; i++)
      fRFToF[i] = -999;
   fTDCRef = 0;
}

AtENCourseEvent::AtENCourseEvent(const AtENCourseEvent &copy) : AtBaseEvent(copy), fTDCRef(copy.fTDCRef)
{
   fF2PPACs = copy.fF2PPACs->Clone();
   fF3PPACs = copy.fF3PPACs->Clone();
   for (int i = 0; i < 4; i++)
      fRFToF[i] = copy.fRFToF[i];
}

AtENCourseEvent &AtENCourseEvent::operator=(AtENCourseEvent object)
{
   swap(*this, object);
   return *this;
}

void AtENCourseEvent::Clear(Option_t *opt)
{
   AtBaseEvent::Clear(opt);
   fF2PPACs.reset();
   fF3PPACs.reset();
   for (int i = 0; i < 4; i++)
      fRFToF[i] = -999;
   fTDCRef = 0;
}
