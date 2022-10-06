#include "AtRemovePulser.h"

#include "AtPad.h"
#include "AtPadBase.h" // for AtPadBase
#include "AtPulserInfo.h"

#include <FairLogger.h>

#include <array>  // for array
#include <memory> // for allocator, make_unique
#include <numeric>

void AtRemovePulser::Filter(AtPad *pad)
{
   addPulserInfo(pad);
   removePulser(pad);
}
void AtRemovePulser::addPulserInfo(AtPad *pad)
{
   auto &adc = pad->GetADC();
   auto pulserInfo = dynamic_cast<AtPulserInfo *>(pad->AddAugment("pulserInfo", std::make_unique<AtPulserInfo>()));

   for (int i = 1; i < adc.size(); ++i) {
      if (std::abs(adc.at(i) - adc.at(i - 1)) > fThreshold) {
         auto [start, stop, mag] = getTransitionAround(pad, i);
         i = stop;
         if (mag > 0) {
            pulserInfo->SetRiseBegin(start);
            pulserInfo->SetRiseEnd(stop);
            pulserInfo->SetRiseMag(mag);
         } else {
            pulserInfo->SetFallBegin(start);
            pulserInfo->SetFallEnd(stop);
            pulserInfo->SetFallMag(-mag);
         }
      }
   }
}

std::tuple<double, double, double> AtRemovePulser::getTransitionAround(AtPad *pad, int idx)
{
   auto &adc = pad->GetADC();

   // Look backwards for when the difference is smaller than
   int start = idx;
   for (; start > 0; start--) {
      if (std::abs(adc[start] - adc[start - 1]) < fThresholdLow)
         break;
   }

   int end = idx;
   for (; end < 512; end++)
      if (std::abs(adc[end] - adc[end - 1]) < fThresholdLow)
         break;

   LOG(debug) << start << " " << end << " " << adc[end] - adc[start];

   return {start, end, adc[end] - adc[start]};
}

void AtRemovePulser::removePulser(AtPad *pad)
{
   auto pulserInfo = dynamic_cast<AtPulserInfo *>(pad->GetAugment("pulserInfo"));
   auto mag =
      std::accumulate(pulserInfo->GetMag().begin(), pulserInfo->GetMag().end(), 0.0) / pulserInfo->GetMag().size();

   // Values for transition region
   auto zeroRise = pad->GetADC(pulserInfo->GetBegin()[0] - 2) + pad->GetADC(pulserInfo->GetBegin()[0] - 1);
   zeroRise /= 2.0;
   auto zeroFall = pad->GetADC(pulserInfo->GetEnd()[1] + 1) + pad->GetADC(pulserInfo->GetEnd()[1]);
   zeroFall /= 2.0;

   for (int i = pulserInfo->GetBegin()[0]; i < pulserInfo->GetEnd()[1]; ++i) {

      if (i < pulserInfo->GetEnd()[0]) {
         pad->SetADC(i, zeroRise);
         continue;
      }
      if (i < pulserInfo->GetBegin()[1]) {
         pad->SetADC(i, pad->GetADC(i) - mag);
         continue;
      }
      pad->SetADC(i, zeroFall);
   }
}
