#include "AtGenericTrace.h"

#include <FairLogger.h>

#include <memory>
#include <string>
#include <utility>

ClassImp(AtGenericTrace);

AtGenericTrace::AtGenericTrace(Int_t traceID, std::size_t size) : fTraceID(traceID), fSize(size)
{
   fRawAdc.resize(size);
   fAdc.resize(size);
}
