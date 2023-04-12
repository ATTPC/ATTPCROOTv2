#ifndef ATPADARRAY_H
#define ATPADARRAY_H

#include "AtPadBase.h" // for AtPadBase

#include <Rtypes.h> // for Double_t, ClassDefOverride

#include <array>  // for array
#include <memory> // for unique_ptr
#include <string>
#include <utility> // for move

class TBuffer;
class TClass;
class TMemberInspector;
class TH1D;

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
   std::unique_ptr<TH1D> GetHist(std::string name) const;
   ClassDefOverride(AtPadArray, 1);
};

#endif //#ifndef ATPADCHARGE_H
