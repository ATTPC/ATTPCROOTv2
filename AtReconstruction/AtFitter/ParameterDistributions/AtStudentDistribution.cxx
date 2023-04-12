#include "AtStudentDistribution.h"

#include <TRandom.h>
namespace MCFitter {

AtStudentDistribution::AtStudentDistribution(double mean, double spread, double seed)
   : AtParameterDistribution(mean, spread, seed)
{
}

double AtStudentDistribution::SampleSpread()
{
   return fDistro(fRand);
}

} // namespace MCFitter
