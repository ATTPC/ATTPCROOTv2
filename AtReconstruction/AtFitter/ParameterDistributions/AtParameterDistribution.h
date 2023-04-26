#ifndef ATPARAMETERDISTRIBUTION_H
#define ATPARAMETERDISTRIBUTION_H

#include <memory>
#include <random>
namespace MCFitter {

class AtParameterDistribution {

protected:
   double fMean{0};
   double fSpread{0};
   long fSeed;
   static thread_local std::unique_ptr<std::mt19937> fRand;

public:
   AtParameterDistribution(double mean, double spread, long seed = 0);
   virtual ~AtParameterDistribution() = default;

   double GetMean() const { return fMean; }
   double GetSpread() const { return fSpread; }

   void SetMean(double mean) { fMean = mean; }
   void SetSpread(double spread) { fSpread = spread; }

   double Sample();

   virtual void TruncateSpace() = 0;

protected:
   virtual double SampleSpread() = 0;
};

} // namespace MCFitter

#endif // ATPARAMETERDISTRIBUTION_H
