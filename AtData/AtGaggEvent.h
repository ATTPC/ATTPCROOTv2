#ifndef AtGAGGEVENT_H
#define AtGAGGEVENT_H

#include "AtBaseEvent.h"

#include <Rtypes.h>

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>
class TBuffer;
class TClass;
class TMemberInspector;

class AtGaggEvent : public AtBaseEvent {
private:
   Double_t fE1[25] = {0};
   Double_t fE2[16] = {0};

   Double_t fADCMax1[25] = {0};
   Double_t fADCMax2[16] = {0};

   Int_t fMultiplicity1 = 0;
   Int_t fMultiplicity2 = 0;

public:
   AtGaggEvent();
   AtGaggEvent(const AtGaggEvent &copy);
   AtGaggEvent(const AtBaseEvent &copy) : AtBaseEvent(copy) { SetName("AtGaggEvent"); }
   AtGaggEvent &operator=(const AtGaggEvent object);
   virtual ~AtGaggEvent() = default;

   friend void swap(AtGaggEvent &first, AtGaggEvent &second)
   {
      using std::swap;
      swap(dynamic_cast<AtBaseEvent &>(first), dynamic_cast<AtBaseEvent &>(second));

      swap(first.fE1, second.fE1);
      swap(first.fE2, second.fE2);

      swap(first.fADCMax1, second.fADCMax1);
      swap(first.fADCMax2, second.fADCMax2);

      swap(first.fMultiplicity1, second.fMultiplicity1);
      swap(first.fMultiplicity2, second.fMultiplicity2);
   }

   void Clear(Option_t *opt = nullptr) override;

   void SetE1(Int_t idx, Double_t value) { fE1[idx] = value; }
   void SetE2(Int_t idx, Double_t value) { fE2[idx] = value; }

   void SetADCMax1(Int_t idx, Double_t value) { fADCMax1[idx] = value; }
   void SetADCMax2(Int_t idx, Double_t value) { fADCMax2[idx] = value; }

   void SetMultiplicity1(Int_t value) { fMultiplicity1 = value; }
   void SetMultiplicity2(Int_t value) { fMultiplicity2 = value; }

   Double_t GetE1(Int_t idx) { return fE1[idx]; }
   Double_t GetE2(Int_t idx) { return fE2[idx]; }

   Double_t GetADCMax1(Int_t idx) { return fADCMax1[idx]; }
   Double_t GetADCMax2(Int_t idx) { return fADCMax2[idx]; }

   Int_t GetMultiplicity1() { return fMultiplicity1; }
   Int_t GetMultiplicity2() { return fMultiplicity2; }

   ClassDefOverride(AtGaggEvent, 1);
};

#endif
