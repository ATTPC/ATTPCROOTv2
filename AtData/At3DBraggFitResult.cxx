#include "At3DBraggFitResult.h"

#include <FairLogger.h>

ClassImp(At3DBraggFitResult);

void At3DBraggFitResult::SetBestFitIndex(Int_t &index)
{
   if (index >= fChi2s.size()) {
      LOG(error) << " At3DBraggFitResult : " << index << " is not a valid index since there is only " << fChi2s.size()
                 << " chi2 entries! Leaving the best fit index unchanged!";
      return;
   }
   fBestFitIndex = index;
}
