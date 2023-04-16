#ifndef ATY_H
#define ATY_H

#include "AtIndependentSample.h" // for AtIndependentSample

#include <vector> // for vector
class AtHit;

namespace RandomSample {

/**
 * @brief Sample AtHits according to charge for Y shaped tracks
 *
 * Two points sampled from the beam region and two outside the beam region.
 * Points outside the beam region have to be closer to the pad plane than the points
 * sampled in the beam region
 * @ingroup AtHitSampling
 */
class AtY : public AtIndependentSample {
protected:
   std::vector<int> fNotBeam; //< List of indicies for outer region of the TPC
   std::vector<int> fBeam;    //< List of indicies for inner region of the TPC

   double fBeamRadius{40}; // Radius of the beam in mm

public:
   virtual std::vector<AtHit> SampleHits(int N) override;
   virtual void SetHitsToSample(const std::vector<const AtHit *> &hits) override;
   virtual std::vector<double> PDF(const AtHit &hit) override { return {1}; }
};
} // namespace RandomSample
#endif // AWEIGHTEDY
