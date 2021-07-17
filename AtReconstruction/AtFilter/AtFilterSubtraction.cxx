#include "AtFilterSubtraction.h"

#include "AtRawEvent.h"
#include "AtMap.h"
#include "AtPad.h"

AtFilterSubtraction::AtFilterSubtraction(AtMap *map, Int_t numCoBos) : fNumberCoBo(numCoBos), fMapping(map)
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
}

void AtFilterSubtraction::InitEvent(AtRawEvent *event)
{
   Clear();

   // Loop through every pad and look for the ch0s
   for (int padNum = 0; padNum < event->GetNumPads(); ++padNum) {
      // Get the pad reference {cobo, asad, aget, ch}
      auto padRef = fMapping->GetPadRef(event->GetPad(padNum)->GetPadNum());
      auto cobo = padRef[0];
      auto asad = padRef[1];
      auto ch = padRef[3];

      /*std::cout << padRef.at(0) << " "
      << padRef.at(1) << " "
      << padRef.at(2) << " "
      << padRef.at(3) << std::endl;
      */

      // If it is a channel zero, add the trace to the average
      // TODO: Check to make sure there is no data in pad and record the number of traces
      // added to sum to do average correctly
      if (ch == 0)
         for (int tb = 0; tb < 512; ++tb) {
            fBaseline.at(cobo)[asad][tb] += event->GetPad(padNum)->GetADC(tb);
            fRawBaseline.at(cobo)[asad][tb] += event->GetPad(padNum)->GetRawADC(tb);
         }
   } // End loop over all pads in TPC
}

void AtFilterSubtraction::Filter(AtPad *pad)
{
   // Get the pad reference
   auto padRef = fMapping->GetPadRef(pad->GetPadNum());
   auto cobo = padRef[0];
   auto asad = padRef[1];

   auto adc = pad->GetADC();
   auto adcRaw = pad->GetRawADC();
   for (int tb = 0; tb < 512; ++tb) {
      adcRaw[tb] -= fRawBaseline[cobo][asad][tb] / 4.0;
      if (pad->IsPedestalSubtracted())
         adc[tb] -= fBaseline[cobo][asad][tb] / 4.0;
   }
}

void AtFilterSubtraction::Init() {}
