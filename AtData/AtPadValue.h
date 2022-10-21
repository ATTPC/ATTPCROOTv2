#ifndef ATPADVALUE_H
#define ATPADVALUE_H

#include "AtPadBase.h" // for AtPadBase

#include <Rtypes.h> // for Double_t, ClassDefOverride

#include <memory> // for unique_ptr

class TBuffer;
class TClass;
class TMemberInspector;

/**
 * @brief Holds a double for an AtPad.
 *
 *
 * @ingroup Pads
 */
class AtPadValue : public AtPadBase {
public:
protected:
   Double_t fValue;

public:
   AtPadValue(Double_t val = 0);
   virtual ~AtPadValue() = default;

   virtual std::unique_ptr<AtPadBase> Clone() const override;

   void SetValue(Double_t val) { fValue = val; }
   Double_t GetValue() const { return fValue; }

   ClassDefOverride(AtPadValue, 1);
};

#endif //#ifndef ATPADCHARGE_H
