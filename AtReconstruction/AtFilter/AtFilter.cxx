#include "AtFilter.h"

#include "AtRawEvent.h"

#include <TClonesArray.h>

AtRawEvent *AtFilter::ConstructOutputEvent(TClonesArray *fOutputEventArray, AtRawEvent *inputEvent)
{
   return new ((*fOutputEventArray)[0]) AtRawEvent(*inputEvent); // NOLINT (ROOT owns memory)
}
