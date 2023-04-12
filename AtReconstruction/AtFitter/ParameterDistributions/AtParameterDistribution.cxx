#include "AtParameterDistribution.h"
namespace MCFitter {
AtParameterDistribution::AtParameterDistribution(double mean, double spread, long seed) : fMean(mean), fSpread(spread)
{
   // Use a random seed if a seed was not passed
   if (seed == 0) {
      std::random_device rd;
      seed = rd();
   }
   fRand = std::mt19937(seed);
}

}// namespace MCFitter
