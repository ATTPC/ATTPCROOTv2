#include "AtIndependentSample.h"

using namespace RandomSample;

void AtIndependentSample::SetHitsToSample(const std::vector<AtHit> *hits)
{
   fHits = hits;
   FillCDF();
}
