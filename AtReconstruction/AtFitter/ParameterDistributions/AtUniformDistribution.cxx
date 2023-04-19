#include "AtUniformDistribution.h"
namespace MCFitter {
AtUniformDistribution::AtUniformDistribution(double mean, double spread, double seed)
   : AtParameterDistribution(mean, spread, seed)
{
}

double AtUniformDistribution::SampleSpread()
{
   return fDistro(fRand);
}

void AtUniformDistribution::TruncateSpace()
{
   fSpread *= fTruncAmount;
}
} // namespace MCFitter
