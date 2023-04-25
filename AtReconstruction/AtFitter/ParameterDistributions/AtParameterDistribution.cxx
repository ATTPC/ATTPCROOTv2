#include "AtParameterDistribution.h"
namespace MCFitter {

thread_local std::unique_ptr<std::mt19937> AtParameterDistribution::fRand = nullptr;

AtParameterDistribution::AtParameterDistribution(double mean, double spread, long seed)
   : fMean(mean), fSpread(spread), fSeed(seed)
{
}

double AtParameterDistribution::Sample()
{
   if (fRand == nullptr) {
      if (fSeed == 0) {
         std::random_device rd;
         fSeed = rd();
      }
      fRand = std::make_unique<std::mt19937>(fSeed);
   }
   return fMean + fSpread * SampleSpread();
}
} // namespace MCFitter
