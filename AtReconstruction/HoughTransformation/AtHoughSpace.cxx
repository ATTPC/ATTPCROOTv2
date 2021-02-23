#include "AtHoughSpace.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(AtHoughSpace)

   AtHoughSpace::AtHoughSpace()
{
   fThreshold = 0.0;
   fHoughDist = 5.0;
   fHoughMaxThreshold = 10.0;
}

AtHoughSpace::~AtHoughSpace() {}

void AtHoughSpace::SetThreshold(Double_t value)
{
   fThreshold = value;
}
void AtHoughSpace::SetHoughDistance(Double_t value)
{
   fHoughDist = value;
}
