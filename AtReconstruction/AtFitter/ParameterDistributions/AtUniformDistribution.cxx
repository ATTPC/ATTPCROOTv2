#include "AtUniformDistribution.h"

#include "AtRandom.h"

#include <TRandom.h>

namespace MCFitter {
AtUniformDistribution::AtUniformDistribution(double mean, double spread) : AtParameterDistribution(mean, spread) {}

double AtUniformDistribution::SampleSpread()
{
   return AtTools::Random::Get().Uniform(-1, 1);
}

void AtUniformDistribution::TruncateSpace()
{
   fSpread *= fTruncAmount;
}
} // namespace MCFitter
