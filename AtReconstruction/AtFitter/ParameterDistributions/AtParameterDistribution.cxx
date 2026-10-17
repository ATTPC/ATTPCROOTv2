#include "AtParameterDistribution.h"

namespace MCFitter {

AtParameterDistribution::AtParameterDistribution(double mean, double spread) : fMean(mean), fSpread(spread) {}

double AtParameterDistribution::Sample()
{
   return fMean + fSpread * SampleSpread();
}
} // namespace MCFitter
