#ifndef ATPSADECONVFIT_H
#define ATPSADECONVFIT_H

#include "AtPSADeconv.h"
#include "AtPad.h" // for AtPad, AtPad::trace

#include <Fit/Fitter.h>
class TH1F;
class TF1;
class AtPSADeconvFit : public AtPSADeconv {
protected:
   double fDiffLong; //< Longitudinal diffusion coefficient

public:
   virtual void Init() override;
   virtual std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSADeconvFit>(*this); }

protected:
   HitData getZandQ(const AtPad::trace &charge) override;

   const ROOT::Fit::FitResult FitHistorgramParallel(TH1F &hist, TF1 &func);
};

#endif // #ifndef ATPSADECONVFIT_H
