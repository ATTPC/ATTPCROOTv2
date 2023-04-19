#include "AtPatternEvent.h"

#include "AtHit.h" // for AtHit

#include <Rtypes.h>

#include <string> // for string

ClassImp(AtPatternEvent);

AtPatternEvent::AtPatternEvent(const char *name) : AtBaseEvent(name) {}

AtPatternEvent::AtPatternEvent(const AtPatternEvent &copy) : AtBaseEvent(copy), fTrackCand(copy.fTrackCand)
{
   for (const auto &hit : copy.fNoise)
      fNoise.push_back(hit->Clone());
}

AtPatternEvent &AtPatternEvent::operator=(AtPatternEvent object)
{
   swap(*this, object);
   return *this;
}

void AtPatternEvent::Clear(Option_t *opt)
{
   AtBaseEvent::Clear(opt);
   fTrackCand.clear();
   fNoise.clear();
}
