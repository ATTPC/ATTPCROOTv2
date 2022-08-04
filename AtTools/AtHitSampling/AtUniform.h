#ifndef ATUNIFORM_H
#define ATUNIFORM_H

#include "AtSample.h"

#include <vector> // for vector
class AtHit;

namespace RandomSample {

/**
 * @brief Uniformly sample a collection of AtHits
 *
 * @ingroup AtHitSampling
 */
class AtUniform : public AtSample {
public:
   virtual std::vector<AtHit> SampleHits(int N) override;
   virtual void SetHitsToSample(const std::vector<const AtHit *> &hits) override { fHits = &hits; }

protected:
   virtual std::vector<double> PDF(const AtHit &hit) override;
};
} // namespace RandomSample
#endif //#ifndef ATSAMPLEUNIFORM_H
