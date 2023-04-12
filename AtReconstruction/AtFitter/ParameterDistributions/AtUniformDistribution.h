#ifndef ATUNIFORMDISTRIBUTION_H
#define ATUNIFORMDISTRIBUTION_H

#include "AtParameterDistribution.h"

#include <random>
namespace MCFitter {

class AtUniformDistribution : public AtParameterDistribution {
protected:
   std::uniform_real_distribution<> fDistro{-1, 1};
   double fTruncAmount{0.8}; //<Default truncation of parameter space from ATTPC commisisoning paper

public:
   AtUniformDistribution(double mean, double spread, double seed = 0);
   virtual ~AtUniformDistribution() = default;

   void TruncateSpace() override;

protected:
   double SampleSpread() override;
};

} // namespace MCFitter

#endif // ATUNIFORMDISTRIBUTION_H
