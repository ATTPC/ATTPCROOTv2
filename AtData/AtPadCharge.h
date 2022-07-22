// Pad object to hold the accumulated electron output of a simulation
#ifndef ATPADCHARGE_H
#define ATPADCHARGE_H

#include "AtPad.h"

#include <Rtypes.h> // for Double_t, ClassDefOverride

#include <array>  // for array
#include <memory> // for unique_ptr
#include <utility>

class TBuffer;
class TClass;
class TMemberInspector;

class AtPadCharge : public AtPad {
public:
   using traceElec = std::array<Double_t, 512>;

protected:
   traceElec fElectrons;

public:
   AtPadCharge(Int_t padNum = -1) : AtPad(padNum) {}
   AtPadCharge(const AtPadCharge &obj) = default;
   AtPadCharge(const AtPad &obj) : AtPad(obj) {}
   virtual ~AtPadCharge() = default;
   virtual std::unique_ptr<AtPad> Clone() override;

   void SetElectrons(const traceElec &val) { fElectrons = val; }
   void SetElectrons(Int_t idx, Double_t val) { fElectrons[idx] = val; }

   const traceElec &GetElectrons() const { return fElectrons; }
   Double_t GetElectrons(Int_t idx) const { return GetElectrons()[idx]; }

   ClassDefOverride(AtPadCharge, 1);
};

#endif //#ifndef ATPADCHARGE_H
