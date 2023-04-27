#ifndef ATESTIMATORMETHODS_H
#define ATESTIMATORMETHODS_H

#include <vector>
class AtHit;
namespace AtPatterns {
class AtPattern;
}
namespace SampleConsensus {

/**
 * @brief Estimators for AtSampleConsensus.
 *
 * All implemented estimators for AtSampleConsensus.
 * @ingroup SampleConsensus
 */
enum class Estimators { kRANSAC, kLMedS, kMLESAC, kWRANSAC, kChi2, kYRANSAC };
/**
 * @brief Implementation of RANSAC estimator.
 *
 * Maximizes the number of inliers.
 */
int EvaluateRansac(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray, double distanceThreshold);

/**
 * @brief Implementation of RANSAC estimator ignoring beam component of y.
 *
 * Maximizes the number of inliers on the non-beam rays of the Y pattern.
 */
int EvaluateYRansac(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray, double distanceThreshold);

/**
 * @brief Implementation of estimator that minimizes chi2.
 *
 * Used to minimize avg(error^2) for all inliers.
 */
int EvaluateChi2(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray, double distanceThreshold);

/**
 * @brief Implementation of MLESAC estimator
 */
int EvaluateMlesac(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray, double distanceThreshold);
/**
 * @brief Implementation of LMedS estimator
 */
int EvaluateLmeds(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray, double distanceThreshold);
/**
 * @brief Implementation of RANSAC estimator using charge weighting
 */
int EvaluateWeightedRansac(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray,
                           double distanceThreshold);

} // namespace SampleConsensus
#endif //#ifndef ATESTIMATORMETHODS_H
