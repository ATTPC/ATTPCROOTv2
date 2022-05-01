#ifndef ATCHARGEWEIGHTED_H
#define ATCHARGEWEIGHTED_H

#include "AtHit.h" // for AtHit
#include "AtIndependentSample.h"

#include <vector> // for vector

namespace RandomSample {

/**
 * @brief Sample AtHits according to charge
 *
 * Follows the PDF: P(q) = q/TotalCharge
 * @ingroup AtHitSampling
 */
class AtChargeWeighted : public AtIndependentSample {

protected:
   virtual std::vector<double> PDF(const AtHit &hit) override;
};
} // namespace RandomSample
#endif // ATCHARGEWEIGHTED
