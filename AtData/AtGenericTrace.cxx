#include "AtGenericTrace.h"

ClassImp(AtGenericTrace);

std::unique_ptr<AtPadBase> AtGenericTrace::Clone() const
{
   return std::make_unique<AtGenericTrace>(*this);
}

AtGenericTrace::AtGenericTrace(Int_t traceID, std::size_t size, std::string na)
   : fTraceID(traceID), fSize(size), fName(na)
{
   fRawAdc.resize(size);
   fAdc.resize(size);
}
