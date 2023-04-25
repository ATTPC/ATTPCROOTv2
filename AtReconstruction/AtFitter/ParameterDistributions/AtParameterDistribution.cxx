#include "AtParameterDistribution.h"
namespace MCFitter {

thread_local std::unique_ptr<std::mt19937> AtParameterDistribution::fRand = nullptr;

AtParameterDistribution::AtParameterDistribution(double mean, double spread, long seed) : fMean(mean), fSpread(spread)
{
   // Use a random seed if a seed was not passed
   if (fSeed == 0) {
      std::random_device rd;
      fSeed = rd();
   }
}

double AtParameterDistribution::Sample()
{
   if (fRand == nullptr)
      fRand = std::make_unique<std::mt19937>(fSeed);
   return fMean + fSpread * SampleSpread();
}
} // namespace MCFitter
