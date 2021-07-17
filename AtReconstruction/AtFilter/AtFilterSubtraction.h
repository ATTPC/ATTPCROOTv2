#ifndef ATFILTERSUBTRACTION_H
#define ATFILTERSUBTRACTION_H

/* Filter that will subtract the average of every ch0 aget in a AsAd from all
   channels in an AsAd.

   Adam Anthony 7/16/21
*/

#include "AtFilter.h"
#include <vector>
#include <array>

class AtMap;
class AtPad;
class AtRawEvent;

using vecDoubleCoBo = std::vector<std::array<std::array<Double_t, 512>, 4>>;
using vecIntCoBo = std::vector<std::array<std::array<Int_t, 512>, 4>>;

class AtFilterSubtraction : public AtFilter {
private:
   const Int_t fNumberCoBo;
   AtMap *fMapping;

   // Avg with baseline subtraction baseline[cobo][asad][tb]
   vecDoubleCoBo fBaseline;
   // Avg without baseline subtraction
   vecIntCoBo fRawBaseline;

   void Clear();

public:
   AtFilterSubtraction(AtMap *map, Int_t numCoBos = 10);

   // Called at the init stage of the AtFilterTask
   virtual void Init() override;

   // Called once for each event at the start of the Exec phase
   virtual void InitEvent(AtRawEvent *event) override;

   // Called on each pad
   virtual void Filter(AtPad *pad) override;
};

#endif //#ifndef ATFILTERSUBTRACTION_H
