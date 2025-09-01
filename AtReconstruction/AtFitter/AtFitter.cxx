#include "AtFitter.h"

ClassImp(AtFITTER::AtFitter);

void AtFITTER::AtFitter::Reset()
{
   fPatternEvent = nullptr;
   fFitMetadata = nullptr;
   fRawEvent = nullptr;
   fEvent = nullptr;
}
