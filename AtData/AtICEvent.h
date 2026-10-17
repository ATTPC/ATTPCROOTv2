#ifndef AtICEVENT_H
#define AtICEVENT_H

#include "AtBaseEvent.h"

#include <Rtypes.h>

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>
class TBuffer;
class TClass;
class TMemberInspector;

class AtICEvent : public AtBaseEvent {
private:
   Double_t fRawADC = -1.;
   Double_t fADC = -1.;
   Bool_t fIsPedestalSubtracted = kFALSE;

public:
   AtICEvent();
   AtICEvent(const AtICEvent &) = default;
   AtICEvent &operator=(const AtICEvent &) = default;
   virtual ~AtICEvent() = default;

   void Clear(Option_t *opt = nullptr) override;

   void SetRawADC(Double_t value) { fRawADC = value; }
   void SetADC(Double_t value) { fADC = value; }
   void SetPedestalSubtracted(Bool_t val = kTRUE) { fIsPedestalSubtracted = val; }

   Double_t GetRawADC() const { return fRawADC; }
   Double_t GetADC() const { return fADC; }
   Bool_t IsPedestalSubtracted() const { return fIsPedestalSubtracted; }

   ClassDefOverride(AtICEvent, 1);
};

#endif
