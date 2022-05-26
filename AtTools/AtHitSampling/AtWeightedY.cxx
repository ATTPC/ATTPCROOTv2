#include "AtWeightedY.h"

#include "AtHit.h"
#include "AtSample.h" // for RandomSample

using namespace RandomSample;

std::vector<AtHit> AtWeightedY::SampleHits(int N)
{
   std::vector<AtHit> ret;
   for (auto ind : sampleIndicesFromCDF(2, fVetoIn))
      ret.push_back(fHits->at(ind));
   for (auto ind : sampleIndicesFromCDF(N - 2, fVetoOut))
      ret.push_back(fHits->at(ind));
   return ret;
}

void AtWeightedY::SetHitsToSample(const std::vector<AtHit> *hits)
{
   fHits = hits;
   FillCDF();
   for (int i = 0; i < fHits->size(); i++) {
      if (sqrt(pow(fHits->at(i).GetPosition().X(), 2) + pow(fHits->at(i).GetPosition().Y(), 2)) < 20) {
         fVetoOut.push_back(i);
      } else {
         fVetoIn.push_back(i);
      }
   }
}
