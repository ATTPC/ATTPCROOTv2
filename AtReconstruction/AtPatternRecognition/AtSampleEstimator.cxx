#include "AtSampleEstimator.h"

#include "AtContainerManip.h"
#include "AtEstimatorMethods.h"
#include "AtHit.h"

using namespace SampleConsensus;

/**
 * @brief Evaluate how well model describes hits
 *
 * Checks all hits for consistancy with modeled pattern, and sets the model Chi2 based on
 * the estimator.
 *
 * @ingroup SampleConsensus
 *
 * @param[in] model Model to evaluate
 * @param[in] hits Hits to compare to model
 * @param[in] distThresh How close a point must be to be consistent with the model
 * @return Number of points consistent with model in hits
 */
int AtEstimator::EvaluateModel(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hits, double distThresh,
                               Estimators estimator = Estimators::kRANSAC)
{
   switch (estimator) {
   case (Estimators::kRANSAC): return EvaluateRansac(model, hits, distThresh);
   case (Estimators::kLMedS): return EvaluateLmeds(model, hits, distThresh);
   case (Estimators::kMLESAC): return EvaluateMlesac(model, hits, distThresh);
   case (Estimators::kWRANSAC): return EvaluateWeightedRansac(model, hits, distThresh);
   case (Estimators::kChi2): return EvaluateChi2(model, hits, distThresh);
   default: return 0;
   }
}

int AtEstimator::EvaluateModel(AtPatterns::AtPattern *model, const std::vector<AtHit> &hits, double distThresh,
                               Estimators estimator = Estimators::kRANSAC)
{
   return EvaluateModel(model, ContainerManip::GetConstPointerVector(hits), distThresh, estimator);
}
