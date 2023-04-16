#include "AtPSAComposite.h"

#include <FairLogger.h>

#include <AtPad.h>

#include <memory>
#include <utility> // for move

AtPSAComposite::AtPSAComposite(std::unique_ptr<AtPSA> beamPSA, std::unique_ptr<AtPSA> PSA, double beamRadius)
   : AtPSA(), fBeamPSA(std::move(beamPSA)), fPSA(std::move(PSA)), fBeamRadius(beamRadius)
{
}

AtPSAComposite::AtPSAComposite(const AtPSAComposite &o)
   : AtPSA(), fBeamPSA(o.fBeamPSA->Clone()), fPSA(o.fPSA->Clone()), fBeamRadius(o.fBeamRadius)
{
}

void AtPSAComposite::Init()
{
   fBeamPSA->Init();
   fPSA->Init();
}
AtPSA::HitVector AtPSAComposite::AnalyzePad(AtPad *pad)
{
   if (pad->GetPadCoord().R() < fBeamRadius) {
      LOG(debug) << "Using beam PSA: " << pad->GetPadCoord().R() << " " << pad->GetPadNum();
      return fBeamPSA->AnalyzePad(pad);
   } else {
      LOG(debug) << "Using PSA: " << pad->GetPadCoord().R() << " " << pad->GetPadNum();
      return fPSA->AnalyzePad(pad);
   }
}
