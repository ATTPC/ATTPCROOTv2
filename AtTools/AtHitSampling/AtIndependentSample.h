#ifndef ATINDEPENDENTSAMPLE_H
#define ATINDEPENDENTSAMPLE_H

#include "AtHit.h"
#include "AtSample.h"

#include <vector>
namespace RandomSample {

/**
 * @brief Interface for independent samples
 *
 * Interface for sampling a collection of AtHits where the points sampled are independent of one another.
 * @ingroup AtHitSampling
 */

class AtIndependentSample : public AtSample {
public:
   virtual ~AtIndependentSample() = default;
   virtual void SetHitsToSample(const std::vector<const AtHit *> &hits) override;

protected:
   virtual std::vector<double> PDF(const AtHit &hit) override = 0;
};
} // namespace RandomSample

#endif //#ifndef ATINDEPENDENTSAMPLE_H
