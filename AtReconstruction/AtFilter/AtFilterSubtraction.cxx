#include "AtFilterSubtraction.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtMap.h"
#include "AtPad.h"
#include "AtPadReference.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <utility>

AtFilterSubtraction::AtFilterSubtraction(AtMapPtr map, Int_t numCoBos, Int_t numAget)
   : fNumberCoBo(numCoBos), fMapping(std::move(map))
{
   fBaseline.resize(fNumberCoBo);
   for (auto &cobo : fBaseline)
      cobo.resize(numAget);
   fRawBaseline.resize(fNumberCoBo);
   for (auto &cobo : fRawBaseline)
      cobo.resize(numAget);
   fAgetCount.resize(fNumberCoBo);
   for (auto &cobo : fAgetCount)
      cobo.resize(numAget);
}

void AtFilterSubtraction::Clear()
{
   // Clear out all of the traces
   for (auto &cobo : fBaseline)
      for (auto &asad : cobo)
         asad.fill(0);
   for (auto &cobo : fRawBaseline)
      for (auto &asad : cobo)
         asad.fill(0);
   for (auto &cobo : fAgetCount)
      for (auto &asad : cobo)
         asad = 0;
   fNumberMissedAsads = 0;
}

void AtFilterSubtraction::InitEvent(AtRawEvent *event)
{
   Clear();

   for (const auto &pad : event->GetPads())
      processPad(fMapping->GetPadRef(pad->GetPadNum()), *pad);
   AverageBaseline();
   fEventNumber = event->GetEventID();
}
/**
 * Should this pad be added to the average to subtract
 */
bool AtFilterSubtraction::isValidPad(const AtPad &pad)
{
   auto padRef = fMapping->GetPadRef(pad.GetPadNum());
   if (padRef.ch != 0)
      return false;

   for (int tb = 0; tb < 512; ++tb)
      if (pad.GetADC(tb) > fThreshold)
         return false;

   return true;
}
int AtFilterSubtraction::getAsad(const AtPadReference &ref)
{
   return ref.asad;
}
void AtFilterSubtraction::processPad(const AtPadReference &ref, const AtPad &pad)
{
   if (isValidPad(pad))
      AddChToBaseline(ref, pad);
}

void AtFilterSubtraction::AddChToBaseline(const AtPadReference &padRef, const AtPad &pad)
{
   int asad = getAsad(padRef);
   fAgetCount[padRef.cobo][asad]++;
   for (int tb = 0; tb < 512; ++tb) {
      fBaseline[padRef.cobo][asad][tb] += pad.GetADC(tb);
      fRawBaseline[padRef.cobo][asad][tb] += pad.GetRawADC(tb);
   }
}

void AtFilterSubtraction::AverageBaseline()
{
   for (int cobo = 0; cobo < fBaseline.size(); ++cobo)
      for (int asad = 0; asad < fBaseline[cobo].size(); ++asad)
         if (fAgetCount[cobo][asad] != 0)
            for (int tb = 0; tb < 512; ++tb) {
               fBaseline[cobo][asad].at(tb) /= fAgetCount[cobo].at(asad);
               fRawBaseline[cobo][asad][tb] /= fAgetCount[cobo][asad];
            }
         else {
            LOG(error) << "No baseline for cobo " << cobo << " asad " << asad << " in event " << fEventNumber;
            fNumberMissedAsads++;
         }
}

void AtFilterSubtraction::Filter(AtPad *pad, AtPadReference *padReference)
{
   // Get the pad reference
   auto padRef = fMapping->GetPadRef(pad->GetPadNum());
   auto cobo = padRef.cobo;
   auto asad = getAsad(padRef);

   auto &adc = pad->GetADC();
   auto &adcRaw = pad->GetRawADC();

   for (int tb = 0; tb < 512; ++tb) {
      // pad->SetRawADC(tb, adcRaw[tb] - fRawBaseline[cobo][asad][tb]);
      if (pad->IsPedestalSubtracted())
         pad->SetADC(tb, adc[tb] - fBaseline[cobo][asad][tb]);
   }
}

void AtFilterSubtraction::Init() {}

bool AtFilterSubtraction::IsGoodEvent()
{
   bool isGoodEvent = (fNumberMissedAsads == 0);
   if (fSetIsGood)
      return isGoodEvent;
   else
      return true;
}
