#include "AtParameterDistribution.h"

#include <FairLogger.h>

namespace MCFitter {

thread_local std::unique_ptr<std::mt19937> AtParameterDistribution::fRand = nullptr;

AtParameterDistribution::AtParameterDistribution(double mean, double spread, long seed)
   : fMean(mean), fSpread(spread), fSeed(seed)
{
}

double AtParameterDistribution::Sample()
{
   long seed = fSeed;
   if (fRand == nullptr) {
      if (seed == 0) {
         std::random_device rd;
         seed = rd();
      }
      LOG(info) << "Seeding thread with " << seed;
      fRand = std::make_unique<std::mt19937>(seed);
   }
   return fMean + fSpread * SampleSpread();
}
} // namespace MCFitter
