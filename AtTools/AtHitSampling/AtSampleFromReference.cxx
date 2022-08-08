#include "AtSampleFromReference.h"

#include "AtHit.h"

#include <TRandom.h> // for TRandom
#include <TRandom3.h>

#include <utility> // for move

using namespace RandomSample;
std::vector<AtHit> AtSampleFromReference::SampleHits(int N)
{
   SampleReferenceHit();

   return AtSample::SampleHits(N);
}

/**
 * @brief Get reference hit from fHits.
 *
 * Uniformly samples from fHits
 */
void AtSampleFromReference::SampleReferenceHit()
{
   int refIndex = gRandom->Uniform() * fHits->size();
   SetReferenceHit(*fHits->at(refIndex));
}

void AtSampleFromReference::SetReferenceHit(AtHit hit)
{
   fReferenceHit = std::move(hit);
   FillCDF();
}
