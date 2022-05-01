#include "AtChargeWeighted.h"

#include "AtHit.h"
#include "AtSample.h" // for RandomSample

using namespace RandomSample;
std::vector<double> AtChargeWeighted::PDF(const AtHit &hit)
{
   return {hit.GetCharge()};
}
