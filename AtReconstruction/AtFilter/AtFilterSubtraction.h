#ifndef ATFILTERSUBTRACTION_H
#define ATFILTERSUBTRACTION_H

/* Filter that will subtract the average of every ch0 aget in a AsAd from all
   channels in an AsAd.

   Adam Anthony 7/16/21
*/

#include "AtFilter.h"
#include <vector>
#include <array>
#include <memory>

// Class forward declerations
class AtMap;
class AtPad;
class AtRawEvent;

using vecDoubleCoBo = std::vector<std::array<std::array<Double_t, 512>, 4>>;
using vecIntCoBo = std::vector<std::array<std::array<Int_t, 512>, 4>>;
using vecAgetCount = std::vector<std::array<int, 4>>;
using AtMapPtr = std::shared_ptr<AtMap>;

class AtFilterSubtraction : public AtFilter {
private:
   const Int_t fNumberCoBo;
   Double_t fThreshold;
   AtMapPtr fMapping;
   Int_t fNumberMissedAsads;

   // Avg with baseline subtraction baseline[cobo][asad][tb]
   vecDoubleCoBo fBaseline;
   // Avg without baseline subtraction
   vecIntCoBo fRawBaseline;
   // Number of AGET ch0s that were used to calculate the baseline
   vecAgetCount fAgetCount;

   void Clear();
   void AddChToBaseline(const AtPad &pad);
   void AverageBaseline();
   void processPad(const AtPad &pad);

public:
   AtFilterSubtraction(AtMapPtr map, Int_t numCoBos = 10);

   void SetThreshold(Double_t thresh) { fThreshold = thresh; }
   Double_t GetThreshold() { return fThreshold; }

   // Called at the init stage of the AtFilterTask
   virtual void Init() override;

   // Called once for each event at the start of the Exec phase
   virtual void InitEvent(AtRawEvent *event) override;

   // Called on each pad
   virtual void Filter(AtPad *pad) override;

   virtual bool IsGoodEvent() override;
};

#endif //#ifndef ATFILTERSUBTRACTION_H
