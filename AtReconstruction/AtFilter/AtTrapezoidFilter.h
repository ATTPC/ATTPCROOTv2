#ifndef ATTRAPEZOIDFILTER_H
#define ATTRAPEZOIDFILTER_H

#include "AtFilter.h"

#include <Rtypes.h>

#include <vector>

class AtPad;
class AtRawEvent;
struct AtPadReference;

class AtTrapezoidFilter : public AtFilter {

private:
   Float_t fM;
   Int_t fRiseTime;
   Int_t fTopTime;
   Int_t fDiscriminatorThreshold;

   // Variables to track this event
   Int_t fStartIndex;
   std::vector<Float_t> d;

   void setSignalStart(AtPad *pad);
   void zeroSignalBeforeStart(AtPad *pad);
   void setDVector(AtPad *pad);

   Float_t p(int index);
   Float_t r(int index);
   Float_t s(int index);

public:
   void SetM(Float_t m) { fM = m; }
   void SetRiseTime(Int_t riseTime) { fRiseTime = riseTime; }
   void SetTopTime(Int_t topTime) { fTopTime = topTime; }
   void SetDiscriminatorThreshold(Int_t threshold) { fDiscriminatorThreshold = threshold; }

   Float_t GetM() { return fM; }
   Int_t GetRiseTime() { return fRiseTime; }
   Int_t GetTopTime() { return fTopTime; }

   virtual void Init() override {}
   virtual void InitEvent(AtRawEvent *event) override {}
   virtual void Filter(AtPad *pad, AtPadReference *padReference) override;
   virtual bool IsGoodEvent() override { return true; }
};

#endif //#ifndef ATTRAPEZOIDFILTER_H
