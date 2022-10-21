#include "AtSCACorrect.h"

#include "AtMap.h"
#include "AtPad.h"
#include "AtPadBase.h"
#include "AtPadValue.h"
#include "AtRawEvent.h"

struct AtPadReference;

void AtSCACorrect::Filter(AtPad *pad, AtPadReference *padReference)
{
   removeBaseline(pad, padReference);
   removePhase(pad, padReference);
}

void AtSCACorrect::removeBaseline(AtPad *pad, AtPadReference *padReference)
{

   AtPad *baselinePad = nullptr;
   if (padReference != nullptr) {
      if (fMap->IsFPNchannel(*padReference)) {
         baselinePad = fBaseline->GetFpn(*padReference);
      } else if (!fMap->IsAuxPad(*padReference)) {
         baselinePad = fBaseline->GetPad(pad->GetPadNum());
      }
   } else {
      baselinePad = fBaseline->GetPad(pad->GetPadNum());
   }

   if (baselinePad != nullptr) {
      for (int i = 0; i < 512; ++i) {
         pad->SetADC(i, pad->GetRawADC(i) - baselinePad->GetADC(i));
      }
      pad->SetPedestalSubtracted(true);
   } else {
      std::cout << "Baseline is null!" << std::endl;
   }
}

void AtSCACorrect::removePhase(AtPad *pad, AtPadReference *padReference)
{

   AtPad *phasePad = nullptr;
   if (padReference != nullptr) {
      if (fMap->IsFPNchannel(*padReference)) {
         phasePad = fPhase->GetFpn(*padReference);
      } else if (!fMap->IsAuxPad(*padReference)) {
         phasePad = fPhase->GetPad(pad->GetPadNum());
      }
   } else {
      phasePad = fPhase->GetPad(pad->GetPadNum());
   }

   if (phasePad != nullptr) {
      int lastCell = (dynamic_cast<AtPadValue *>(pad->GetAugment("lastCell")))->GetValue();
      for (int i = 0; i < 512; i++) {
         int phaseShift = i + lastCell;
         if (phaseShift > 511) {
            phaseShift = phaseShift - 512;
         }
         pad->SetADC(i, pad->GetADC(i) - phasePad->GetADC(phaseShift));
      }
   } else {
      std::cout << "Phase is null!" << std::endl;
   }
}
