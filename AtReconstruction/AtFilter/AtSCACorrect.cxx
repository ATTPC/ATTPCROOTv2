#include "AtSCACorrect.h"

#include "AtMap.h"
#include "AtPad.h"
#include "AtPadArray.h"
#include "AtPadBase.h"
#include "AtPadValue.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <TFile.h> // for TFile

#include <utility> // for move

struct AtPadReference;

AtSCACorrect::AtSCACorrect(AtMapPtr map, TString filename, TString eventName, TString baselineAug, TString phaseAug)
   : AtFilter(), fMap(std::move(map)), fDoBaseline(true), fDoPhase(true), fBaseAugName(baselineAug),
     fPhaseAugName(phaseAug), fUseChanZero(false)
{
   TFile f1(filename.Data());
   AtRawEvent *unownedEvent = nullptr;
   f1.GetObject(eventName.Data(), unownedEvent);
   fRawEvent = RawEventPtr(unownedEvent);
   if (fRawEvent != nullptr) {
      LOG(info) << "baseline raw event opened";
   }
}

AtSCACorrect::AtSCACorrect(AtMapPtr map, RawEventPtr rawEvent, TString baselineAug, TString phaseAug)
   : AtFilter(), fMap(std::move(map)), fRawEvent(std::move(rawEvent)), fDoBaseline(true), fDoPhase(true),
     fBaseAugName(baselineAug), fPhaseAugName(phaseAug), fUseChanZero(false)
{
}

void AtSCACorrect::Filter(AtPad *pad, AtPadReference *padReference)
{
   if (fDoBaseline)
      removeBaseline(pad, padReference);
   if (fDoPhase)
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

   auto padRef = *padReference;
   if (fUseChanZero)
      padRef.ch = 0;

   AtPad *baselinePad = getMatchingPad(pad, &padRef, fRawEvent.get());

   if (baselinePad != nullptr) {
      auto baseArray = dynamic_cast<AtPadArray *>(baselinePad->GetAugment(fBaseAugName.Data()));
      if (baseArray != nullptr) {
         for (int i = 0; i < 512; ++i) {
            pad->SetADC(i, pad->GetRawADC(i) - baseArray->GetArray(i));
         }
         pad->SetPedestalSubtracted(true);
      } else {
         LOG(error) << "Baseline Array is null!";
      }
   } else {
      // LOG(error) << "Baseline is null!";
   }
}

void AtSCACorrect::removePhase(AtPad *pad, AtPadReference *padReference)
{
   auto padRef = *padReference;
   if (fUseChanZero)
      padRef.ch = 0;

   AtPad *phasePad = getMatchingPad(pad, &padRef, fRawEvent.get());

   if (phasePad != nullptr) {
      auto phaseArray = dynamic_cast<AtPadArray *>(phasePad->GetAugment(fBaseAugName.Data()));
      if (phaseArray != nullptr) {
         int lastCell = (dynamic_cast<AtPadValue *>(pad->GetAugment("lastCell")))->GetValue();
         for (int i = 0; i < 512; i++) {
            int phaseShift = i + lastCell - 1;
            if (phaseShift > 511) {
               phaseShift = phaseShift - 512;
            }
            pad->SetADC(i, pad->GetADC(i) - phaseArray->GetArray(phaseShift));
         }
      } else {
         LOG(error) << "Phase Array is null!";
      }
   } else {
      // LOG(error) << "Phase is null!";
   }
}
