#include "AtENCourseEvent.h"

#include <Rtypes.h>

ClassImp(AtENCourseEvent);

AtENCourseEvent::AtENCourseEvent() : AtBaseEvent("AtENCourseEvent") {}

AtENCourseEvent::AtENCourseEvent(const AtENCourseEvent &copy)
   : AtBaseEvent(copy)
{
   fF2PPACs = copy.fF2PPACs->Clone();
   fF3PPACs = copy.fF3PPACs->Clone();
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
}
