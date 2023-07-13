#include "AtGenericTrace.h"

ClassImp(AtGenericTrace);

AtGenericTrace::AtGenericTrace(Int_t traceID, std::size_t size) : fTraceID(traceID), fSize(size)
{
   fRawAdc.resize(size);
   fAdc.resize(size);
}
