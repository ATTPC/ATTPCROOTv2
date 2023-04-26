#include "AtUniformDistribution.h"

#include <memory> // for unique_ptr<>::element_type, unique_ptr
#include <random> // for uniform_real_distribution

namespace MCFitter {
AtUniformDistribution::AtUniformDistribution(double mean, double spread, double seed)
   : AtParameterDistribution(mean, spread, seed)
{
}

double AtUniformDistribution::SampleSpread()
{
   // N.B. We recreate the distribution (which is cheap) each call to avoid data races when multithreaded
   std::uniform_real_distribution<> distro{-1, 1};
   return distro(*fRand);
}

void AtUniformDistribution::TruncateSpace()
{
   fSpread *= fTruncAmount;
}
} // namespace MCFitter
