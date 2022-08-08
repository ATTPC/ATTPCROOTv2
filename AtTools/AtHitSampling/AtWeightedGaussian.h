#ifndef ATWEIGHTEDGAUSSIAN_H
#define ATWEIGHTEDGAUSSIAN_H

#include "AtChargeWeighted.h"
#include "AtHit.h" // for AtHit
#include "AtSampleFromReference.h"

#include <vector> // for vector

namespace RandomSample {

/**
 * @brief Sample with a charge-weighted gaussian.
 *
 * Class for sampling a collection of AtHits in a spacial gaussian distribution around
 * the reference hit where the hits are weighted by charge.
 *
 * @ingroup AtHitSampling
 */
class AtWeightedGaussian : public AtSampleFromReference {
protected:
   double fSigma; //< Sigma of gaussian around fReferencehit to sample [mm]
   AtChargeWeighted fChargeSample;

public:
   AtWeightedGaussian(double sigma = 30) : fSigma(sigma) {}
   virtual void SetHitsToSample(const std::vector<const AtHit *> &hits) override;

protected:
   virtual std::vector<double> PDF(const AtHit &hit) override;
   virtual void SampleReferenceHit() override;
};
} // namespace RandomSample
#endif // ATSAMPLEGAUSSIAN_H
