#ifndef AtSIEVENT_H
#define AtSIEVENT_H

#include "AtBaseEvent.h"

#include <Rtypes.h>

#include <memory>
#include <type_traits>
#include <utility>
#include <vector>
class TBuffer;
class TClass;
class TMemberInspector;

class AtSiEvent : public AtBaseEvent {
private:
   Double_t fEFront1[4] = {-1, -1, -1, -1};
   Double_t fEBack1[4] = {-1, -1, -1, -1};
   Double_t fEFront2[4] = {-1, -1, -1, -1};
   Double_t fEBack2[4] = {-1, -1, -1, -1};

   Int_t fTSFront1[4] = {-1, -1, -1, -1};
   Int_t fTSBack1[4] = {-1, -1, -1, -1};
   Int_t fTSFront2[4] = {-1, -1, -1, -1};
   Int_t fTSBack2[4] = {-1, -1, -1, -1};

   Double_t fADCMaxFront1[4] = {-1, -1, -1, -1};
   Double_t fADCMaxBack1[4] = {-1, -1, -1, -1};
   Double_t fADCMaxFront2[4] = {-1, -1, -1, -1};
   Double_t fADCMaxBack2[4] = {-1, -1, -1, -1};

   Int_t fStripFront1[4] = {-1, -1, -1, -1};
   Int_t fStripBack1[4] = {-1, -1, -1, -1};
   Int_t fStripFront2[4] = {-1, -1, -1, -1};
   Int_t fStripBack2[4] = {-1, -1, -1, -1};

   Int_t fMultiplicityFront1 = 0;
   Int_t fMultiplicityBack1 = 0;
   Int_t fMultiplicityFront2 = 0;
   Int_t fMultiplicityBack2 = 0;

   Int_t fXStrip1 = -1; // positioning of strips class
   Int_t fXStrip2 = -1;
   Int_t fYStrip1 = -1;
   Int_t fYStrip2 = -1;
   Double_t fEnergyFront1 = -1; // storing energy of strips
   Double_t fEnergyFront2 = -1;
   Double_t fEnergyBack1 = -1;
   Double_t fEnergyBack2 = -1;
   Int_t fTimeFront1 = -1; // timestamps for coincidences
   Int_t fTimeFront2 = -1;
   Int_t fTimeBack1 = -1;
   Int_t fTimeBack2 = -1;

public:
   AtSiEvent();
   AtSiEvent(const AtSiEvent &copy);
   AtSiEvent(const AtBaseEvent &copy) : AtBaseEvent(copy) { SetName("AtSiEvent"); }
   AtSiEvent &operator=(const AtSiEvent object);
   virtual ~AtSiEvent() = default;

   friend void swap(AtSiEvent &first, AtSiEvent &second)
   {
      using std::swap;
      swap(dynamic_cast<AtBaseEvent &>(first), dynamic_cast<AtBaseEvent &>(second));

      swap(first.fEFront1, second.fEFront1);
      swap(first.fEBack1, second.fEBack1);
      swap(first.fEFront2, second.fEFront2);
      swap(first.fEBack2, second.fEBack2);

      swap(first.fTSFront1, second.fTSFront1);
      swap(first.fTSBack1, second.fTSBack1);
      swap(first.fTSFront2, second.fTSFront2);
      swap(first.fTSBack2, second.fTSBack2);

      swap(first.fADCMaxFront1, second.fADCMaxFront1);
      swap(first.fADCMaxBack1, second.fADCMaxBack1);
      swap(first.fADCMaxFront2, second.fADCMaxFront2);
      swap(first.fADCMaxBack2, second.fADCMaxBack2);

      swap(first.fStripFront1, second.fStripFront1);
      swap(first.fStripBack1, second.fStripBack1);
      swap(first.fStripFront2, second.fStripFront2);
      swap(first.fStripBack2, second.fStripBack2);

      swap(first.fMultiplicityFront1, second.fMultiplicityFront1);
      swap(first.fMultiplicityBack1, second.fMultiplicityBack1);
      swap(first.fMultiplicityFront2, second.fMultiplicityFront2);
      swap(first.fMultiplicityBack2, second.fMultiplicityBack2);

      swap(first.fTimeFront1, second.fTimeFront1);
      swap(first.fTimeBack1, second.fTimeBack1);
      swap(first.fTimeFront2, second.fTimeFront2);
      swap(first.fTimeBack2, second.fTimeBack2);
   }

   void Clear(Option_t *opt = nullptr) override;

   void SetEFront1(Int_t idx, Double_t value) { fEFront1[idx] = value; }
   void SetEBack1(Int_t idx, Double_t value) { fEBack1[idx] = value; }
   void SetEFront2(Int_t idx, Double_t value) { fEFront2[idx] = value; }
   void SetEBack2(Int_t idx, Double_t value) { fEBack2[idx] = value; }

   void SetTSFront1(Int_t idx, Int_t value) { fTSFront1[idx] = value; }
   void SetTSBack1(Int_t idx, Int_t value) { fTSBack1[idx] = value; }
   void SetTSFront2(Int_t idx, Int_t value) { fTSFront2[idx] = value; }
   void SetTSBack2(Int_t idx, Int_t value) { fTSBack2[idx] = value; }

   void SetADCMaxFront1(Int_t idx, Double_t value) { fADCMaxFront1[idx] = value; }
   void SetADCMaxBack1(Int_t idx, Double_t value) { fADCMaxBack1[idx] = value; }
   void SetADCMaxFront2(Int_t idx, Double_t value) { fADCMaxFront2[idx] = value; }
   void SetADCMaxBack2(Int_t idx, Double_t value) { fADCMaxBack2[idx] = value; }

   void SetStripFront1(Int_t idx, Int_t value) { fStripFront1[idx] = value; }
   void SetStripBack1(Int_t idx, Int_t value) { fStripBack1[idx] = value; }
   void SetStripFront2(Int_t idx, Int_t value) { fStripFront2[idx] = value; }
   void SetStripBack2(Int_t idx, Int_t value) { fStripBack2[idx] = value; }

   void SetMultiplicityFront1(Int_t value) { fMultiplicityFront1 = value; }
   void SetMultiplicityBack1(Int_t value) { fMultiplicityBack1 = value; }
   void SetMultiplicityFront2(Int_t value) { fMultiplicityFront2 = value; }
   void SetMultiplicityBack2(Int_t value) { fMultiplicityBack2 = value; }

   Double_t GetEFront1(Int_t idx) { return fEFront1[idx]; }
   Double_t GetEBack1(Int_t idx) { return fEBack1[idx]; }
   Double_t GetEFront2(Int_t idx) { return fEFront2[idx]; }
   Double_t GetEBack2(Int_t idx) { return fEBack2[idx]; }

   Int_t GetTSFront1(Int_t idx) { return fTSFront1[idx]; }
   Int_t GetTSBack1(Int_t idx) { return fTSBack1[idx]; }
   Int_t GetTSFront2(Int_t idx) { return fTSFront2[idx]; }
   Int_t GetTSBack2(Int_t idx) { return fTSBack2[idx]; }

   Double_t GetADCMaxFront1(Int_t idx) { return fADCMaxFront1[idx]; }
   Double_t GetADCMaxBack1(Int_t idx) { return fADCMaxBack1[idx]; }
   Double_t GetADCMaxFront2(Int_t idx) { return fADCMaxFront2[idx]; }
   Double_t GetADCMaxBack2(Int_t idx) { return fADCMaxBack2[idx]; }

   Int_t GetStripFront1(Int_t idx) { return fStripFront1[idx]; }
   Int_t GetStripBack1(Int_t idx) { return fStripBack1[idx]; }
   Int_t GetStripFront2(Int_t idx) { return fStripFront2[idx]; }
   Int_t GetStripBack2(Int_t idx) { return fStripBack2[idx]; }

   Int_t GetMultiplicityFront1() { return fMultiplicityFront1; }
   Int_t GetMultiplicityBack1() { return fMultiplicityBack1; }
   Int_t GetMultiplicityFront2() { return fMultiplicityFront2; }
   Int_t GetMultiplicityBack2() { return fMultiplicityBack2; }

   Int_t GetXStrip1() { return fXStrip1; }
   Int_t GetXStrip2() { return fXStrip2; }
   Int_t GetYStrip1() { return fYStrip1; }
   Int_t GetYStrip2() { return fYStrip2; }

   Int_t GetTimeFront1() { return fTimeFront1; }
   Int_t GetTimeFront2() { return fTimeFront2; }
   Int_t GetTimeBack1() { return fTimeBack1; }
   Int_t GetTimeBack2() { return fTimeBack2; }

   Double_t GetEnergyFront1() { return fEnergyFront1; }
   Double_t GetEnergyFront2() { return fEnergyFront2; }
   Double_t GetEnergyBack1() { return fEnergyBack1; }
   Double_t GetEnergyBack2() { return fEnergyBack2; }

   void BuildHits();

   ClassDefOverride(AtSiEvent, 1);
};

#endif
