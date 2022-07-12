#include "AtWeightedY.h"

#include "AtHit.h"
#include "AtSample.h" // for RandomSample

#include <algorithm> // for max
#include <cmath>     // for pow, sqrt
#include <memory>    // for allocator_traits<>::value_type

using namespace RandomSample;

std::vector<AtHit> AtWeightedY::SampleHits(int N)
{
   std::vector<AtHit> ret;

   // If every hit is outside of the beam region, then skip the vetoed beam region
   if (fVetoIn.size() == fHits->size() || fVetoIn.size() == 0) {
      for (auto ind : sampleIndicesFromCDF(N))
         ret.push_back(fHits->at(ind));
      return ret;
   }

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
