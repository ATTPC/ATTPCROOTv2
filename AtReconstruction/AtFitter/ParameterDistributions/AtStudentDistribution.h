#ifndef ATSTUDENTDISTRIBUTION_H
#define ATSTUDENTDISTRIBUTION_H

#include "AtParameterDistribution.h"

#include <random>
namespace MCFitter {

class AtStudentDistribution : public AtParameterDistribution {

protected:
   std::student_t_distribution<> fDistro{1};

public:
   AtStudentDistribution(double mean, double spread, double seed = 0);
   virtual ~AtStudentDistribution() = default;

   double GetDoF() const { return fDistro.n(); }
   void SetDoF(double dof) { fDistro = std::student_t_distribution<>(dof); }

   void TruncateSpace() override { fSpread *= .5; }

protected:
   double SampleSpread() override;
};

} // namespace MCFitter

#endif // ATSTUDENTDISTRIBUTION_H
