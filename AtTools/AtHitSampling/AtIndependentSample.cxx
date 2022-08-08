#include "AtIndependentSample.h"

using namespace RandomSample;

void AtIndependentSample::SetHitsToSample(const std::vector<const AtHit *> &hits)
{
   fHits = &hits;
   FillCDF();
}
