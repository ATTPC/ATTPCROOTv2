#ifndef ATPULSERINFO_H
#define ATPULSERINFO_H
#include "AtPadBase.h"

#include <Rtypes.h> // for Int_t, THashConsistencyHolder, Double_t, Clas...

#include <array>  // for array
#include <memory> // for make_unique, unique_ptr

class TBuffer;
class TClass;
class TMemberInspector;

class AtPulserInfo : public AtPadBase {
private:
   using IntArray = std::array<Int_t, 2>;
   using DoubleArray = std::array<Int_t, 2>;

   IntArray fBegin{-1, -1};
   IntArray fEnd{-1, -1};
   DoubleArray fMag{-1, -1};

public:
   void SetRiseBegin(Int_t val) { fBegin[0] = val; }
   void SetFallBegin(Int_t val) { fBegin[1] = val; }
   void SetRiseEnd(Int_t val) { fEnd[0] = val; }
   void SetFallEnd(Int_t val) { fEnd[1] = val; }
   void SetRiseMag(Double_t val) { fMag[0] = val; }
   void SetFallMag(Double_t val) { fMag[1] = val; }

   const IntArray &GetBegin() { return fBegin; }
   const IntArray &GetEnd() { return fEnd; }
   const IntArray &GetMag() { return fMag; }

   virtual std::unique_ptr<AtPadBase> Clone() const override { return std::make_unique<AtPulserInfo>(*this); }

   ClassDefOverride(AtPulserInfo, 1)
};

#endif // ATPULSERINFO_H
