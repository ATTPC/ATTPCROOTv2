#ifndef ATFILTER_H
#define ATFILTER_H
// Interface for filters that can be applied to the raw signal traces

#include "Rtypes.h"

#include <vector>

class AtFilter {

public:
   // Called at the init stage of the AtFilterTask
   virtual void Init() = 0;

   // Called on each pad if no baseline subtraction
   virtual void Filter(Int_t *trace) = 0;

   // Called on each pad if baseline subtraction
   virtual void Filter(Double_t *trace) = 0;
};

#endif //#ifndef ATFILTER_H
