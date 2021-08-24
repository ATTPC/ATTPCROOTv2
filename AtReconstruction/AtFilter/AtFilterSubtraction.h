#ifndef ATFILTERSUBTRACTION_H
#define ATFILTERSUBTRACTION_H

/* Filter that will subtract the average of every ch0 aget in a AsAd from all
   channels in an AsAd.

   Adam Anthony 7/16/21
*/

#include "AtFilter.h"
#include <vector>
#include <array>

// Class forward declerations
class AtMap;
class AtPad;
class AtRawEvent;

using vecDoubleCoBo = std::vector<std::array<std::array<Double_t, 512>, 4>>;
using vecIntCoBo = std::vector<std::array<std::array<Int_t, 512>, 4>>;
using vecAgetCount = std::vector<std::array<int, 4>>;

class AtFilterSubtraction : public AtFilter {
private:
   const Int_t fNumberCoBo;
   Double_t fThreshold;
   AtMap *fMapping;

   // Avg with baseline subtraction baseline[cobo][asad][tb]
   vecDoubleCoBo fBaseline;
   // Avg without baseline subtraction
   vecIntCoBo fRawBaseline;
   // Number of AGET ch0s that were used to calculate the baseline
   vecAgetCount fAgetCount;

   void Clear();
   void AddChToBaseline(AtPad *pad);
   void AverageBaseline();

public:
   AtFilterSubtraction(AtMap *map, Int_t numCoBos = 10);

   void SetThreshold(Double_t thresh) { fThreshold = thresh; }
   Double_t GetThreshold() { return fThreshold; }

   // Called at the init stage of the AtFilterTask
   virtual void Init() override;

   // Called once for each event at the start of the Exec phase
   virtual void InitEvent(AtRawEvent *event) override;

   // Called on each pad
   virtual void Filter(AtPad *pad) override;
};

#endif //#ifndef ATFILTERSUBTRACTION_H
