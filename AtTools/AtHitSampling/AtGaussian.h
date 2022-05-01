#ifndef ATGAUSSIAN_H
#define ATGAUSSIAN_H
#include "AtHit.h" // for AtHit
#include "AtSampleFromReference.h"

#include <vector> // for vector

namespace RandomSample {

/**
 * @brief Sample AtHits in gaussian from reference.
 *
 * Class for sampling a collection of AtHits in a spacial gaussian distribution around
 * the reference hit.
 *
 * @ingroup AtHitSampling
 */
class AtGaussian : public AtSampleFromReference {
protected:
   double fSigma; //< Sigma of gaussian around fReferencehit to sample [mm]

public:
   AtGaussian(double sigma = 30) : fSigma(sigma) {}

protected:
   virtual std::vector<double> PDF(const AtHit &hit) override;
};
} // namespace RandomSample
#endif // ATSAMPLEGAUSSIAN_H
