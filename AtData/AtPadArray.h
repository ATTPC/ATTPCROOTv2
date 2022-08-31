#ifndef ATPADARRAY_H
#define ATPADARRAY_H

#include "AtPad.h"

#include <Rtypes.h> // for Double_t, ClassDefOverride

#include <array>  // for array
#include <memory> // for unique_ptr

class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief Holds an addition array of doubles for an AtPad.
 *
 *
 * @ingroup Pads
 */
class AtPadArray : public AtPadBase {
public:
   using traceDouble = std::array<Double_t, 512>;

protected:
   traceDouble fArray;

public:
   virtual std::unique_ptr<AtPadBase> Clone() const override;

   void SetArray(traceDouble val) { fArray = std::move(val); }
   void SetArray(Int_t idx, Double_t val) { fArray.at(idx) = val; }

   const traceDouble &GetArray() const { return fArray; }
   Double_t GetArray(Int_t idx) const { return fArray[idx]; }

   ClassDefOverride(AtPadArray, 1);
};

#endif //#ifndef ATPADCHARGE_H
