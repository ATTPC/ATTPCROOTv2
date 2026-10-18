#ifndef ATPSADECONVFIT_H
#define ATPSADECONVFIT_H

#include "AtPSADeconv.h"
#include "AtPad.h" // for AtPad, AtPad::trace

#include <memory> // for unique_ptr, make_unique
class AtPSA;

class TH1F;
class AtPSADeconvFit : public AtPSADeconv {
protected:
   double fDiffLong; //< Longitudinal diffusion coefficient
   static thread_local std::unique_ptr<TH1F> fHist;

public:
   virtual void Init() override;
   virtual std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSADeconvFit>(*this); }

protected:
   HitData getZandQ(const AtPad::trace &charge) override;

   /// Fit a gaussian (amplitude, mean, sigma) to hist over [xmin, xmax], starting from par. Returns false if the fit
   /// failed. Otherwise par holds the fitted values.
   static bool FitGaussian(const TH1F &hist, double xmin, double xmax, double (&par)[3]);
};

#endif // #ifndef ATPSADECONVFIT_H
