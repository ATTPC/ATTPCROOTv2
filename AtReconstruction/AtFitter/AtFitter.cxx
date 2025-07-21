#include "AtFitter.h"

ClassImp(AtFITTER::AtFitter);

void AtFITTER::AtFitter::Reset()
{
   fPatternEvent = nullptr;
   fFitResult = nullptr;
   fRawEvent = nullptr;
   fEvent = nullptr;
}
