// Pad object to hold the output of a FFT
#ifndef ATPADFFT_H
#define ATPADFFT_H

#include "AtPadBase.h"

#include <Rtypes.h> // for Double_t, THashConsistencyHolder, ClassDefOverride
#include <TComplex.h>

#include <array>  // for array
#include <memory> // for unique_ptr
#include <utility>

class TVirtualFFT;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPadFFT : public AtPadBase {
public:
   using TraceTrans = std::array<Double_t, 512 / 2 + 1>;

protected:
   // Output of FFT (stored in hermitian symmetric array)
   TraceTrans fRe;
   TraceTrans fIm;

public:
   virtual std::unique_ptr<AtPadBase> Clone() const override;

   Double_t GetPointRe(int i) const;
   Double_t GetPointIm(int i) const;
   Double_t GetPointMag(int i) const;
   Double_t GetPointPhase(int i) const;
   TComplex GetPointComplex(int i) const { return {GetPointRe(i), GetPointIm(i)}; }
   std::pair<Double_t, Double_t> GetPoint(int i) const { return {GetPointRe(i), GetPointIm(i)}; }

   void SetPointRe(int i, Double_t val);
   void SetPointIm(int i, Double_t val);
   void SetPoint(int i, TComplex val);
   void SetData(TraceTrans re, TraceTrans im);
   void GetDataFromFFT(const TVirtualFFT *fft);
   void SetFFTData(TVirtualFFT *fft);

   static std::unique_ptr<AtPadFFT> CreateFromFFT(const TVirtualFFT *fft);
   ClassDefOverride(AtPadFFT, 1);
};

#endif //#ifndef ATPADFFT_H
