#ifndef ATFILTERSUBTRACTION_H
#define ATFILTERSUBTRACTION_H

#include "AtFilter.h"

#include <Rtypes.h>

#include <array>
#include <memory>
#include <vector>

// Class forward declerations
class AtMap;
class AtPad;
class AtRawEvent;
class AtPadReference;

using vecDoubleCoBo = std::vector<std::vector<std::array<Double_t, 512>>>;
using vecIntCoBo = std::vector<std::vector<std::array<Int_t, 512>>>;
using vecAgetCount = std::vector<std::vector<Int_t>>;
using AtMapPtr = std::shared_ptr<AtMap>;

/**
 * Filter that will subtract the average of every ch0 aget in a AsAd from all
 * channels in an AsAd.
 * Adam Anthony 7/16/21
 * @ingroup RawFilters
 */
class AtFilterSubtraction : public AtFilter {
protected:
   const Int_t fNumberCoBo;
   Int_t fEventNumber{-1};
   Double_t fThreshold = 0;
   AtMapPtr fMapping;
   Int_t fNumberMissedAsads{};
   Bool_t fSetIsGood{false}; // if true will set the IsGood flag on the filtered AtRawEvent

   // Avg with baseline subtraction baseline[cobo][asad][tb]
   vecDoubleCoBo fBaseline;
   // Avg without baseline subtraction
   vecIntCoBo fRawBaseline;
   // Number of AGET ch0s that were used to calculate the baseline
   vecAgetCount fAgetCount;

   void Clear();
   void AddChToBaseline(const AtPadReference &ref, const AtPad &pad);
   void AverageBaseline();
   void processPad(const AtPadReference &ref, const AtPad &pad);

   virtual bool isValidPad(const AtPad &pad);
   virtual int getAsad(const AtPadReference &ref);

public:
   AtFilterSubtraction(AtMapPtr map, Int_t numCoBos = 10, Int_t numAget = 4);

   void SetThreshold(Double_t thresh) { fThreshold = thresh; }
   /// Set if we should mark the event bad if we cannot find every AGET to do subtraction
   void SetIsGood(Bool_t val) { fSetIsGood = val; }
   Double_t GetThreshold() const { return fThreshold; }

   // Called at the init stage of the AtFilterTask
   virtual void Init() override;

   // Called once for each event at the start of the Exec phase
   virtual void InitEvent(AtRawEvent *event) override;

   // Called on each pad
   virtual void Filter(AtPad *pad, AtPadReference *padReference) override;

   virtual bool IsGoodEvent() override;
};

#endif //#ifndef ATFILTERSUBTRACTION_H
