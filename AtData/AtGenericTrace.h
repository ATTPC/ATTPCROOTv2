#ifndef AtGENERICTRACE_H
#define AtGENERICTRACE_H

#include "AtPadBase.h"

#include <Rtypes.h>
#include <TObject.h>

#include <cstddef> // for size_t
#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief Trace recorded by other data acquisition systems
 *
 */

class AtGenericTrace : public AtPadBase {
public:
   using rawTrace = std::vector<Int_t>;
   using trace = std::vector<Double_t>;

protected:
   Int_t fTraceID{};
   std::size_t fSize{};
   rawTrace fRawAdc{};
   trace fAdc{};

public:
   virtual std::unique_ptr<AtPadBase> Clone() const override;

   AtGenericTrace(Int_t traceID = -1, std::size_t size = 2048);

   void SetRawADC(Int_t idx, Int_t val) { fRawAdc[idx] = val; }
   void SetADC(Int_t idx, Double_t val) { fAdc[idx] = val; }

   rawTrace &GetRawADC() { return fRawAdc; }
   trace &GetADC() { return fAdc; }
   Int_t GetTraceSize() const { return fSize; }

   ClassDefOverride(AtGenericTrace, 1);
};

#endif
