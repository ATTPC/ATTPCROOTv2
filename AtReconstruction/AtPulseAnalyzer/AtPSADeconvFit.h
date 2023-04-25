#ifndef ATPSADECONVFIT_H
#define ATPSADECONVFIT_H

#include "AtPSADeconv.h"
#include "AtPad.h" // for AtPad, AtPad::trace

class AtPSADeconvFit : public AtPSADeconv {

public:
   virtual void Init() override;

protected:
   double fDiffLong; //< Longitudinal diffusion coefficient

   HitData getZandQ(const AtPad::trace &charge) override;
};

#endif // #ifndef ATPSADECONVFIT_H
