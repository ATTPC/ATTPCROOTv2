#include "AtUniform.h"

#include "AtHit.h"
#include "AtSample.h" // for RandomSample

#include <TRandom.h>

#include <algorithm>
using namespace RandomSample;

std::vector<AtHit> AtUniform::SampleHits(int N)
{
   std::vector<int> ind;
   std::vector<AtHit> retVec;
   while (ind.size() < N) {
      int i = gRandom->Uniform() * fHits->size();
      if (fWithReplacement || !isInVector(i, ind)) {
         ind.push_back(i);
         retVec.push_back(*fHits->at(i));
      }
   }
   return retVec;
}
std::vector<double> AtUniform::PDF(const AtHit &hit)
{
   return {};
}
