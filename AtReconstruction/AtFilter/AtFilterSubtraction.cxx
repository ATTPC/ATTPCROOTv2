#include "AtFilterSubtraction.h"
#include "FairLogger.h"
#include "AtRawEvent.h"
#include "AtMap.h"
#include "AtPad.h"

#include <algorithm>

AtFilterSubtraction::AtFilterSubtraction(AtMapPtr map, Int_t numCoBos)
   : fNumberCoBo(numCoBos), fMapping(map), fThreshold(0)
{
   fBaseline.resize(fNumberCoBo);
   fRawBaseline.resize(fNumberCoBo);
   fAgetCount.resize(fNumberCoBo);
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
      processPad(pad);

   AverageBaseline();

   // LOG(INFO) << "AtFilterSubtraction: Used " << fNumberChUsed << " channels out of a possible " << fNumberChAvail;
}

void AtFilterSubtraction::processPad(const AtPad &pad)
{
   auto padRef = fMapping->GetPadRef(pad.GetPadNum());
   if (padRef.ch != 0)
      return;

   for (int tb = 0; tb < 512; ++tb)
      if (pad.GetADC(tb) > fThreshold)
         return;

   AddChToBaseline(pad);
}

void AtFilterSubtraction::AddChToBaseline(const AtPad &pad)
{
   auto padRef = fMapping->GetPadRef(pad.GetPadNum());
   fAgetCount[padRef.cobo][padRef.asad]++;
   for (int tb = 0; tb < 512; ++tb) {
      fBaseline[padRef.cobo][padRef.asad][tb] += pad.GetADC(tb);
      fRawBaseline[padRef.cobo][padRef.asad][tb] += pad.GetRawADC(tb);
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
            LOG(ERROR) << "All AGET ch0s had data for cobo " << cobo << " asad " << asad;
            fNumberMissedAsads++;
         }
}

void AtFilterSubtraction::Filter(AtPad *pad)
{
   // Get the pad reference
   auto padRef = fMapping->GetPadRef(pad->GetPadNum());
   auto cobo = padRef.cobo;
   auto asad = padRef.asad;
   auto adc = pad->GetADC();
   auto adcRaw = pad->GetRawADC();

   for (int tb = 0; tb < 512; ++tb) {
      adcRaw[tb] -= fRawBaseline[cobo][asad][tb];
      if (pad->IsPedestalSubtracted())
         adc[tb] -= fBaseline[cobo][asad][tb];
   }
}

void AtFilterSubtraction::Init() {}

bool AtFilterSubtraction::IsGoodEvent()
{
   return fNumberMissedAsads == 0;
}
