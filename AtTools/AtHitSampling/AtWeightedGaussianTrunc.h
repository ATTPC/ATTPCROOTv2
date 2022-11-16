#ifndef ATWEIGHTEDGAUSSIANTRUNC_H
#define ATWEIGHTEDGAUSSIANTRUNC_H

#include "AtSample.h"

#include <vector> // for vector
class AtHit;

namespace RandomSample {

/**
 * @brief Uniformly sample a collection of AtHits
 *
 * @ingroup AtHitSampling
 */
class AtWeightedGaussianTrunc : public AtSample {
public:
   virtual std::vector<AtHit> SampleHits(int N) override;
   virtual void SetHitsToSample(const std::vector<const AtHit *> &hits) override { fHits = &hits; }

protected:
   virtual std::vector<double> PDF(const AtHit &hit) override;
};
} // namespace RandomSample
#endif //#ifndef ATWEIGHTEDGAUSSIANTRUNC_H
