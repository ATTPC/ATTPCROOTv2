#include "AtWeightedY.h"

#include "AtHit.h"
#include "AtSample.h" // for RandomSample

#include <FairLogger.h> // for Logger, LOG

#include <algorithm> // for max
#include <cmath>     // for pow, sqrt

using namespace RandomSample;

std::vector<AtHit> AtWeightedY::SampleHits(int N)
{
   std::vector<AtHit> ret;
   LOG(debug) << "Vetoing " << fVetoIn.size();

   // If every hit is outside of the beam region, then skip the vetoed beam region
   if (fVetoIn.size() == fHits->size() || fVetoIn.size() == 0) {
      LOG(error) << "Defaulting to normal sampling (fVetoIn is size: " << fVetoIn.size() << ")";
      for (auto ind : sampleIndicesFromCDF(N))
         ret.push_back(*fHits->at(ind));
      return ret;
   }

   for (auto ind : sampleIndicesFromCDF(2, fVetoIn))
      ret.push_back(*fHits->at(ind));
   for (auto ind : sampleIndicesFromCDF(N - 2, fVetoOut))
      ret.push_back(*fHits->at(ind));
   return ret;
}

void AtWeightedY::SetHitsToSample(const std::vector<const AtHit *> &hits)
{
   fHits = &hits;
   fVetoOut.clear();
   fVetoIn.clear();
   FillCDF();
   for (int i = 0; i < fHits->size(); i++) {
      if (sqrt(pow(fHits->at(i)->GetPosition().X(), 2) + pow(fHits->at(i)->GetPosition().Y(), 2)) < 20) {
         fVetoOut.push_back(i);
      } else {
         fVetoIn.push_back(i);
      }
   }
}
