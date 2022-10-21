#ifndef ATREMOVEPULSER_H
#define ATREMOVEPULSER_H
#include "AtFilter.h"

#include <tuple>
class AtPad;
class AtRawEvent;
struct AtPadReference;

class AtRemovePulser : public AtFilter {
private:
   double fThreshold;
   double fThresholdLow;

public:
   AtRemovePulser(double threshold, double thresholdLow)
      : AtFilter(), fThreshold(threshold), fThresholdLow(thresholdLow)
   {
   }

   virtual void Init() override {}
   virtual void InitEvent(AtRawEvent *) override {}
   virtual bool IsGoodEvent() override { return true; }

   virtual void Filter(AtPad *pad, AtPadReference *padReference) override;

private:
   std::tuple<double, double, double> getTransitionAround(AtPad *pad, int idx);
   void removePulser(AtPad *pad);
   void addPulserInfo(AtPad *pad);
};

#endif //#ifndef ATREMOVEPULSER_H
