#ifndef ATSAMPLEMETHODS_H
#define ATSAMPLEMETHODS_H

#include "AtChargeWeighted.h"        // IWYU pragma: keep
#include "AtGaussian.h"              // IWYU pragma: keep
#include "AtSample.h"                // IWYU pragma: keep
#include "AtUniform.h"               // IWYU pragma: keep
#include "AtWeightedGaussian.h"      // IWYU pragma: keep
#include "AtWeightedGaussianTrunc.h" // IWYU pragma: keep
#include "AtWeightedY.h"             // IWYU pragma: keep
#include "AtY.h"                     // IWYU pragma: keep

#include <memory>
#include <utility>
namespace RandomSample {
/**
 * @brief. Methods of random sampling.
 *
 * All methods implemented that can be constructed by the factory method CreateSampler(SampleMethod).
 * @ingroup AtHitSampling
 */
enum class SampleMethod {
   kUniform = 0,
   kChargeWeighted = 1,
   kGaussian = 2,
   kWeightedGaussian = 3,
   kWeightedY = 4,
   kWeightedGaussianTrunc = 5,
   kY = 6
};

/**
 * @brief. Create a hit sampler
 *
 * Create a sampler using the method, and any parameters required by the type's constructor.
 * @param[in] method SampleMethod to create
 * @param[in] params Arguments to forward to the constructor of method
 * @ingroup AtHitSampling
 */
template <typename... Ts>
std::unique_ptr<AtSample> CreateSampler(SampleMethod method, Ts &&... params)
{
   switch (method) {
   case SampleMethod::kUniform: return std::make_unique<AtUniform>();
   case SampleMethod::kChargeWeighted: return std::make_unique<AtChargeWeighted>();
   case SampleMethod::kGaussian: return std::make_unique<AtGaussian>(std::forward<Ts>(params)...);
   case SampleMethod::kWeightedGaussian: return std::make_unique<AtWeightedGaussian>(std::forward<Ts>(params)...);
   case SampleMethod::kWeightedY: return std::make_unique<AtWeightedY>(std::forward<Ts>(params)...);
   case SampleMethod::kWeightedGaussianTrunc: return std::make_unique<AtWeightedGaussianTrunc>();
   case SampleMethod::kY: return std::make_unique<AtY>();
   default: return nullptr;
   };
}
} // namespace RandomSample

#endif // #ifndef ATSAMPLEMETHODS_H
