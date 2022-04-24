// Pad object to hold the output of a FFT
#ifndef ATPADFFT_H
#define ATPADFFT_H

#include "AtPad.h"

#include <Rtypes.h> // for Double_t, THashConsistencyHolder, ClassDefOverride

#include <array>  // for array
#include <memory> // for unique_ptr
class TBuffer;
class TClass;
class TMemberInspector;

#include <utility>

class TVirtualFFT;

class AtPadFFT : public AtPad {
public:
   using TraceTrans = std::array<Double_t, 512 / 2 + 1>;

protected:
   // Output of FFT (stored in hermitian symmetric array)
   TraceTrans fRe;
   TraceTrans fIm;

public:
   AtPadFFT(Int_t padNum = -1) : AtPad(padNum) {}
   AtPadFFT(const AtPadFFT &obj) = default;
   AtPadFFT(const AtPad &obj) : AtPad(obj) {}
   virtual ~AtPadFFT() = default;
   virtual std::unique_ptr<AtPad> Clone() override;

   Double_t GetPointRe(int i);
   Double_t GetPointIm(int i);
   Double_t GetPointMag(int i);
   Double_t GetPointPhase(int i);
   std::pair<Double_t, Double_t> GetPoint(int i) { return {GetPointRe(i), GetPointIm(i)}; }

   void SetPointRe(int i, Double_t val);
   void SetPointIm(int i, Double_t val);
   void SetData(TraceTrans re, TraceTrans im);
   void GetDataFromFFT(TVirtualFFT *fft);
   void SetFFTData(TVirtualFFT *fft);

   ClassDefOverride(AtPadFFT, 1);
};

#endif //#ifndef ATPADFFT_H
