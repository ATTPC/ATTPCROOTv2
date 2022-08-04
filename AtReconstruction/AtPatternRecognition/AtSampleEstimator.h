#ifndef ATSAMPLEESTIMATOR_H
#define ATSAMPLEESTIMATOR_H

#include <vector>
namespace AtPatterns {
class AtPattern;
}
class AtHit;

namespace SampleConsensus {
enum class Estimators;

/**
 * Static class for calling the correct estimator based on the enum
 * Enum definition and implementation is in AtEstimatorMethods.h
 * @ingroup AtSampleEstimator
 */
class AtEstimator {
public:
   static int
   EvaluateModel(AtPatterns::AtPattern *model, const std::vector<AtHit> &hits, double distThresh, Estimators estimator);
   static int EvaluateModel(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hits, double distThresh,
                            Estimators estimator);
};
} // namespace SampleConsensus

#endif //#ifndef ATSAMPLEESTIMATOR_H
