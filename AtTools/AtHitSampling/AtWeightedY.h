#ifndef ATWEIGHTEDY_H
#define ATWEIGHTEDY_H

#include "AtChargeWeighted.h"

#include <vector> // for vector
class AtHit;

namespace RandomSample {

/**
 * @brief Sample AtHits according to charge for Y shaped tracks
 *
 * Follows the PDF: P(q) = q/TotalCharge
 * @ingroup AtHitSampling
 */
class AtWeightedY : public AtChargeWeighted {
protected:
   std::vector<int> fVetoIn;  //< List of indicies for outer region of the TPC
   std::vector<int> fVetoOut; //< List of indicies for inner region of the TPC

public:
   virtual std::vector<AtHit> SampleHits(int N) override;
   virtual void SetHitsToSample(const std::vector<const AtHit *> &hits) override;
};
} // namespace RandomSample
#endif // AWEIGHTEDY
