#include "AtSCACorrect.h"

#include "AtMap.h"
#include "AtPad.h"
#include "AtPadReference.h"
#include "AtPadValue.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <numeric>
void AtSCACorrect::Filter(AtPad *pad, AtPadReference *padReference)
{
   removeBaseline(pad, padReference);
   removePhase(pad, padReference);
}

void AtSCACorrect::removeBaseline(AtPad *pad, AtPadReference *padReference)
{

   AtPad *baselinePad;
   if (padReference != nullptr) {
      if (fMap->IsFPNchannel(*padReference)) {
         baselinePad = fBaseline->GetFpn(*padReference);
      } else if (!fMap->IsAuxPad(*padReference)) {
         baselinePad = fBaseline->GetPad(pad->GetPadNum());
      }
   } else {
      baselinePad = fBaseline->GetPad(pad->GetPadNum());
   }

   for (int i = 0; i < 512; ++i) {
      pad->SetADC(i, pad->GetRawADC(i) - baselinePad->GetADC(i));
   }
   pad->SetPedestalSubtracted(kTRUE);
}

void AtSCACorrect::removePhase(AtPad *pad, AtPadReference *padReference)
{

   AtPad *phasePad;
   if (padReference != nullptr) {
      if (fMap->IsFPNchannel(*padReference)) {
         phasePad = fPhase->GetFpn(*padReference);
      } else if (!fMap->IsAuxPad(*padReference)) {
         phasePad = fPhase->GetPad(pad->GetPadNum());
      }
   } else {
      phasePad = fPhase->GetPad(pad->GetPadNum());
   }

   int lastCell = (dynamic_cast<AtPadValue *>(pad->GetAugment("lastCell")))->GetValue();
   for (int i = 0; i < 512; i++) {
      int phaseShift = i + lastCell;
      if (phaseShift > 511) {
         phaseShift = phaseShift - 512;
      }
      pad->SetADC(i, pad->GetADC(i) - phasePad->GetADC(phaseShift));
   }
}
