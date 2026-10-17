#include "AtICEvent.h"

ClassImp(AtICEvent)

   AtICEvent::AtICEvent()
   : AtBaseEvent("AtICEvent")
{
}

void AtICEvent::Clear(Option_t *opt)
{
   AtBaseEvent::Clear(opt);

   fRawADC = -1.;
   fADC = -1.;
   fIsPedestalSubtracted = kFALSE;
}
