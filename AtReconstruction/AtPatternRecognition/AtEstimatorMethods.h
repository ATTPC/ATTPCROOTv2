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
enum class Estimators { kRANSAC, kLMedS, kMLESAC };

/**
 * @brief Implementation of RANSAC estimator
 */
int EvaluateRansac(AtPatterns::AtPattern *model, const std::vector<AtHit> &hitArray, double distanceThreshold);
/**
 * @brief Implementation of MLESAC estimator
 */
int EvaluateMlesac(AtPatterns::AtPattern *model, const std::vector<AtHit> &hitArray, double distanceThreshold);
/**
 * @brief Implementation of LMedS estimator
 */
int EvaluateLmeds(AtPatterns::AtPattern *model, const std::vector<AtHit> &hitArray, double distanceThreshold);

} // namespace SampleConsensus
#endif //#ifndef ATESTIMATORMETHODS_H
