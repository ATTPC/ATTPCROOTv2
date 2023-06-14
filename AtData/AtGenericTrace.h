#ifndef AtGENERICTRACE_H
#define AtGENERICTRACE_H

#include <Rtypes.h>
#include <TObject.h>

#include <array>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

/**
 * @brief Trace recorded by other data acquisition systems
 *
 */

class AtGenericTrace : public TObject {
public:
   using rawTrace = std::vector<Int_t>;
   using trace = std::vector<Double_t>;

protected:
   Int_t fTraceID{};
   std::size_t fSize{};
   rawTrace fRawAdc{};
   trace fAdc{};

public:
   AtGenericTrace(Int_t traceID = -1, std::size_t size = 2048);
   AtGenericTrace(AtGenericTrace &&) = default;
   virtual ~AtGenericTrace() = default;

   void SetRawADC(Int_t idx, Int_t val) { fRawAdc[idx] = val; }
   void SetADC(Int_t idx, Double_t val) { fAdc[idx] = val; }

   const rawTrace &GetRawADC() const { return fRawAdc; }
   const trace &GetADC() const { return fAdc; }

   ClassDefOverride(AtGenericTrace, 1);
};

#endif
