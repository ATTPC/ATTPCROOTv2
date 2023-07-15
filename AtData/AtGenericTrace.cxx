#include "AtGenericTrace.h"

ClassImp(AtGenericTrace);

std::unique_ptr<AtPadBase> AtGenericTrace::Clone() const
{
   return std::make_unique<AtGenericTrace>(*this);
}

AtGenericTrace::AtGenericTrace(Int_t traceID, std::size_t size) : fTraceID(traceID), fSize(size)
{
   fRawAdc.resize(size);
   fAdc.resize(size);
}
