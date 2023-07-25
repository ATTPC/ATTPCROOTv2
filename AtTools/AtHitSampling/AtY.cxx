#include "AtY.h"

#include "AtHit.h"
#include "AtSample.h" // for RandomSample

#include <FairLogger.h> // for Logger, LOG

#include <algorithm> // for max
#include <memory>    // for allocator_traits<>::value_type

using namespace RandomSample;

std::vector<AtHit> AtY::SampleHits(int N)
{
   std::vector<AtHit> ret;
   LOG(debug) << "Vetoing " << fBeam.size() << " from beam region";

   // If every hit is outside of the beam region, then skip the vetoed beam region
   if (fBeam.size() + N - 2 > fHits->size() || fNotBeam.size() + 2 > fHits->size()) {
      LOG(debug) << "Defaulting to normal sampling (fBeam is size: " << fBeam.size() << ")";
      for (auto ind : sampleIndicesFromCDF(N))
         ret.push_back(*fHits->at(ind));
      return ret;
   }

   double maxZ = std::max(ret[0].GetPosition().Z(), ret[1].GetPosition().Z());

   // Try to sample 10 times respecting all of the conditions.
   // If they're not met just return the final attempt
   for (int iter = 0; iter < 10; iter++) {
      ret.clear();

      // Sample the beam region
      for (auto ind : sampleIndicesFromCDF(2, fNotBeam))
         ret.push_back(*fHits->at(ind));

      // try 10 times to find fragment hits that are closer to the pad plane
      for (int iterFF = 0; iterFF < 10; ++iterFF) {
         auto indices = sampleIndicesFromCDF(N - 2, fBeam);
         bool isGood = true;
         for (auto ind : indices)
            isGood |= fHits->at(ind)->GetPosition().Z() > maxZ;

         if (isGood) {
            for (auto ind : indices)
               ret.push_back(*fHits->at(ind));
            return ret;
         }
      }
   }

   // Failed to find hits that meat our contition but we should have beam sampled already
   // so just sample the non-beam region.
   for (auto ind : sampleIndicesFromCDF(N - 2, fBeam))
      ret.push_back(*fHits->at(ind));

   return ret;
}

void AtY::SetHitsToSample(const std::vector<const AtHit *> &hits)
{
   fHits = &hits;
   fBeam.clear();
   fNotBeam.clear();
   FillCDF();
   for (int i = 0; i < fHits->size(); i++) {
      if (fHits->at(i)->GetPosition().Rho() < fBeamRadius) {
         fBeam.push_back(i);
      } else {
         fNotBeam.push_back(i);
      }
   }
}
