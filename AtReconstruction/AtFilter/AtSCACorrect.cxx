#include "AtSCACorrect.h"

#include "AtMap.h"
#include "AtPad.h"
#include "AtPadBase.h"
#include "AtPadValue.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <TFile.h> // for TFile

#include <utility> // for move

struct AtPadReference;

AtSCACorrect::AtSCACorrect(AtMapPtr map, TString baselineFilename, TString baselineEventName, TString phaseFilename,
                           TString phaseEventName)
   : AtFilter(), fMap(std::move(map))
{
   TFile f1(baselineFilename.Data());
   AtRawEvent *unownedEvent = nullptr;
   f1.GetObject(baselineEventName.Data(), unownedEvent);
   fBaseline = RawEventPtr(unownedEvent);
   if (fBaseline != nullptr) {
      LOG(info) << "baseline opened";
   }

   TFile f2(phaseFilename.Data());
   f2.GetObject(phaseEventName.Data(), unownedEvent);
   fPhase = RawEventPtr(unownedEvent);
   if (fPhase != nullptr) {
      LOG(info) << "phase opened";
   }
}

AtSCACorrect::AtSCACorrect(AtMapPtr map, RawEventPtr baseline, RawEventPtr phase)
   : AtFilter(), fMap(std::move(map)), fBaseline(std::move(baseline)), fPhase(std::move(phase))
{
}

void AtSCACorrect::Filter(AtPad *pad, AtPadReference *padReference)
{
   removeBaseline(pad, padReference);
   removePhase(pad, padReference);
}

AtPad *AtSCACorrect::getMatchingPad(AtPad *pad, AtPadReference *padReference, AtRawEvent *event)
{

   if (padReference != nullptr) {
      if (fMap->IsFPNchannel(*padReference))
         return event->GetFpn(*padReference);
      if (!fMap->IsAuxPad(*padReference))
         return event->GetPad(pad->GetPadNum());
      return nullptr;
   }
   return event->GetPad(pad->GetPadNum());
}

void AtSCACorrect::removeBaseline(AtPad *pad, AtPadReference *padReference)
{

   AtPad *baselinePad = getMatchingPad(pad, padReference, fBaseline.get());

   if (baselinePad != nullptr) {
      for (int i = 0; i < 512; ++i) {
         pad->SetADC(i, pad->GetRawADC(i) - baselinePad->GetADC(i));
      }
      pad->SetPedestalSubtracted(true);
   } else {
      LOG(error) << "Baseline is null!";
   }
}

void AtSCACorrect::removePhase(AtPad *pad, AtPadReference *padReference)
{

   AtPad *phasePad = getMatchingPad(pad, padReference, fPhase.get());

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
      LOG(error) << "Phase is null!";
   }
}
