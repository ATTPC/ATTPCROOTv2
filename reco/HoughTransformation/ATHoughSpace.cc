#include "ATHoughSpace.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(ATHoughSpace)

ATHoughSpace::ATHoughSpace()
{
  fThreshold = 0.0;
  fHoughDist = 5.0;
}

ATHoughSpace::~ATHoughSpace()
{
}

void ATHoughSpace::SetThreshold(Double_t value)     {fThreshold = value;}
void ATHoughSpace::SetHoughDistance(Double_t value) {fHoughDist = value;}
