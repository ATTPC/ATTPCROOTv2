#include "AtTrapezoidFilter.h"

#include "AtPad.h"

struct AtPadReference;

void AtTrapezoidFilter::Filter(AtPad *pad, AtPadReference *padReference)
{
   setSignalStart(pad);
   zeroSignalBeforeStart(pad);
   setDVector(pad);

   for (int i = fStartIndex; i < 512; ++i)
      pad->SetADC(i, s(i));
}

void AtTrapezoidFilter::setSignalStart(AtPad *pad)
{
   fStartIndex = 72;
}

void AtTrapezoidFilter::zeroSignalBeforeStart(AtPad *pad)
{
   for (int i = 0; i < fStartIndex; ++i) {
      pad->SetADC(i, 0);
      pad->SetRawADC(i, 0);
   }
}

void AtTrapezoidFilter::setDVector(AtPad *pad)
{
   d.clear();
   for (int i = 0; i < 512; ++i) {
      d.push_back(pad->GetADC(i));
      if (i - fRiseTime >= 0)
         d.back() -= pad->GetADC(i - fRiseTime);
      if (i - fTopTime >= 0)
         d.back() -= pad->GetADC(i - fTopTime);
      if (i - fRiseTime - fTopTime >= 0)
         d.back() += pad->GetADC(i - fTopTime - fRiseTime);
   }
}

Float_t AtTrapezoidFilter::r(int index)
{
   return p(index) + fM * d.at(index);
}
Float_t AtTrapezoidFilter::p(int index)
{
   if (index < 0)
      return 0;
   return p(index - 1) + d.at(index);
}
Float_t AtTrapezoidFilter::s(int index)
{
   if (index < 0)
      return 0;
   return s(index - 1) + r(index);
}
