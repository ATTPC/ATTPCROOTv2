#ifndef ATPARAMETERDISTRIBUTION_H
#define ATPARAMETERDISTRIBUTION_H

namespace MCFitter {

/**
 * Base class of the fitter's parameter distributions. Samples are drawn from
 * AtTools::Random::Get(), the calling thread's generator (gRandom if none is installed).
 */
class AtParameterDistribution {

protected:
   double fMean{0};
   double fSpread{0};

public:
   AtParameterDistribution(double mean, double spread);
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
