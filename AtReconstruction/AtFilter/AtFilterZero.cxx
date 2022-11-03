#include "AtFilterZero.h"

#include "AtPad.h"

struct AtPadReference;

void AtFilterZero::Filter(AtPad *pad, AtPadReference *padReference)
{
   bool inZeroRange = false;
   int zeroStart = -1;

   for (int i = 0; i < 512; ++i) {

      if (pad->GetRawADC(i) == 0 && !inZeroRange) {
         inZeroRange = true;
         zeroStart = i;
         continue;
      }

      if (inZeroRange && pad->GetRawADC(i) != 0) {
         // We just left a zero region fill the missing data
         fillMissingData(pad, zeroStart, i);
         // fillMissingDataLine(pad, zeroStart, i);
         inZeroRange = false;
         zeroStart = -1;
      }
   }
}
/// Folling normal convensions, start is inclusive, stop is exclusive.
void AtFilterZero::fillMissingData(AtPad *pad, int start, int stop)
{
   double avg = pad->GetRawADC(start - 1) + pad->GetRawADC(stop);
   avg /= 2.0;

   double avgSub = pad->GetADC(start - 1) + pad->GetADC(stop);
   avgSub /= 2.0;

   double transition = std::abs(pad->GetRawADC(start - 1) - pad->GetRawADC(stop));

   if (transition > fThreshold) {
      if (start >= 1) {
         avg = pad->GetRawADC(start - 1);
         avgSub = pad->GetADC(start - 1);
      } else {
         avg = pad->GetRawADC(stop);
         avgSub = pad->GetADC(stop);
      }
   }

   // Now we need to fill the missing data
   for (int i = start; i < stop; ++i) {
      pad->SetADC(i, avgSub);
      pad->SetRawADC(i, avg);
   }
}

/// Folling normal convensions, start is inclusive, stop is exclusive.
void AtFilterZero::fillMissingDataLine(AtPad *pad, int start, int stop)
{
   double slope = pad->GetADC(stop) - pad->GetADC(start - 1);
   slope /= (stop - start + 1);

   auto startVal = pad->GetADC(start - 1);
   // Now we need to fill the missing data
   for (int i = start; i < stop; ++i) {

      pad->SetADC(i, startVal + (i - start + 1) * slope);
   }
}
