#ifndef ATSAMPLEFROMREFERENCE_H
#define ATSAMPLEFROMREFERENCE_H

#include "AtHit.h"
#include "AtSample.h"

#include <vector>
namespace RandomSample {

/**
 * @brief Sample when PDF depends on reference
 *
 * Interface for sampling a collection of AtHits where the PDF depends on
 * a reference hit.
 *
 * @ingroup AtHitSampling
 */
class AtSampleFromReference : public AtSample {
protected:
   AtHit fReferenceHit; //< Hit to use to construct the CDF/PDF

public:
   virtual ~AtSampleFromReference() = default;
   virtual std::vector<AtHit> SampleHits(int N) override;
   virtual void SetHitsToSample(const std::vector<const AtHit *> &hits) override { fHits = &hits; }
   void SetReferenceHit(AtHit hit);
   const AtHit &GetReferenceHit() const { return fReferenceHit; }

protected:
   virtual void SampleReferenceHit();
};
} // namespace RandomSample
#endif //#ifndef ATSAMPLEFROMREFERENCE_H
