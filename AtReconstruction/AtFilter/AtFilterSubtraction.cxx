#include "AtFilterSubtraction.h"

#include "AtRawEvent.h"
#include "AtMap.h"
#include "AtPad.h"

AtFilterSubtraction::AtFilterSubtraction(AtMap *map, Int_t numCoBos)
   : fNumberCoBo(numCoBos), fMapping(map), fThreshold(0)
{
   fBaseline.resize(fNumberCoBo);
   fRawBaseline.resize(fNumberCoBo);
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
}

void AtFilterSubtraction::InitEvent(AtRawEvent *event)
{
   Clear();

   for (int i = 0; i < event->GetNumPads(); ++i) {
      auto pad = event->GetPad(i);
      auto padRef = fMapping->GetPadRef(pad->GetPadNum());

      if (padRef.ch == 0 && fThreshold < pad->GetADC(pad->GetMaxADCIdx()))
         AddChToBaseline(event->GetPad(i));
   }

   AverageBaseline();
}

void AtFilterSubtraction::AddChToBaseline(AtPad *pad)
{
   auto padRef = fMapping->GetPadRef(pad->GetPadNum());
   fAgetCount[padRef.cobo][padRef.asad]++;
   for (int tb = 0; tb < 512; ++tb) {
      fBaseline.at(padRef.cobo)[padRef.asad][tb] += pad->GetADC(tb);
      fRawBaseline.at(padRef.cobo)[padRef.asad][tb] += pad->GetRawADC(tb);
   }
}

void AtFilterSubtraction::AverageBaseline()
{
   for (int cobo = 0; cobo < fBaseline.size(); ++cobo)
      for (int asad = 0; asad < fBaseline[cobo].size(); ++asad)
         for (int tb = 0; tb < 512; ++tb) {
            fBaseline[cobo][asad][tb] /= fAgetCount[cobo][asad];
            fRawBaseline[cobo][asad][tb] /= fAgetCount[cobo][asad];
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
