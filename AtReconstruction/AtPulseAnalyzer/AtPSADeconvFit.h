#ifndef ATPSADECONVFIT_H
#define ATPSADECONVFIT_H

#include "AtPSADeconv.h"
#include "AtPad.h" // for AtPad, AtPad::trace

#include <Fit/Fitter.h>
class TH1F;
class TF1;
class AtPSADeconvFit : public AtPSADeconv {

public:
   virtual void Init() override;
   virtual std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSADeconvFit>(*this); }

protected:
   double fDiffLong; //< Longitudinal diffusion coefficient

   HitData getZandQ(const AtPad::trace &charge) override;

   const ROOT::Fit::FitResult *FitHistorgramParallel(TH1D &hist, TF1 &func);
};

#endif // #ifndef ATPSADECONVFIT_H
