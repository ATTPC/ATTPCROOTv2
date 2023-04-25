#include "AtStudentDistribution.h"

namespace MCFitter {

AtStudentDistribution::AtStudentDistribution(double mean, double spread, double seed)
   : AtParameterDistribution(mean, spread, seed)
{
}

double AtStudentDistribution::SampleSpread()
{
   // N.B. We recreate the distribution (which is cheap) each call to avoid data races when multithreaded
   std::student_t_distribution<> distro{1};
   return distro(*fRand);
}

} // namespace MCFitter
