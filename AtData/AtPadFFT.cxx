#include "AtPadFFT.h"

#include <TComplex.h>
#include <TVirtualFFT.h>

#include <cassert>
#include <cmath>

std::unique_ptr<AtPadBase> AtPadFFT::Clone() const
{
   return std::make_unique<AtPadFFT>(*this);
}

/**
 * @brief Returns the real part of the ith frequency compnent.
 */
Double_t AtPadFFT::GetPointRe(int i) const
{

   if (i < fRe.size())
      return fRe[i];
   else
      return fRe.at(512 - i);
}

/**
 * @brief Returns the imaginary part of the ith frequency compnent.
 */
Double_t AtPadFFT::GetPointIm(int i) const
{
   if (i < fIm.size())
      return fIm[i];
   else
      return -fIm.at(512 - i);
}
/**
 * @brief Returns the magnitude of the ith frequency compnent.
 */
Double_t AtPadFFT::GetPointMag(int i) const
{
   return std::sqrt(GetPointRe(i) * GetPointRe(i) + GetPointIm(i) * GetPointIm(i));
}
/**
 * @brief Returns the phase of the ith frequency compnent (-pi,pi].
 */
Double_t AtPadFFT::GetPointPhase(int i) const
{
   return std::atan2(GetPointIm(i), GetPointRe(i));
}
/**
 * @brief Sets the real value of the ith frequency component.
 */
void AtPadFFT::SetPointRe(int i, Double_t val)
{
   if (i < fRe.size())
      fRe[i] = val;
   else
      fRe.at(512 - i) = val;
}
/**
 * @brief Sets the imaginary value of the ith frequency component.
 */
void AtPadFFT::SetPointIm(int i, Double_t val)
{
   if (i < fIm.size())
      fIm[i] = val;
   else
      fIm.at(512 - i) = -val;
}

/**
 * @brief Sets the real and imaginary parts of all frequency components.
 */
void AtPadFFT::SetData(TraceTrans re, TraceTrans im)
{
   fRe = std::move(re);
   fIm = std::move(im);
}

/**
 * @brief Sets the real and imaginary parts of all frequency components from the TVirtualFFT.
 *
 */
void AtPadFFT::GetDataFromFFT(const TVirtualFFT *fft)
{
   assert(fft->GetN()[0] / 2 + 1 == fRe.size());

   for (int i = 0; i < fRe.size(); ++i)
      fft->GetPointComplex(i, fRe.at(i), fIm.at(i));
   /*
   for (auto &re : fRe)
      re /= fft->GetN()[0];
   for (auto &im : fIm)
      im /= fft->GetN()[0];
   */
}
/**
 * @brief Sets the real and imaginary parts of all frequency components in a TVirtualFFT.
 */
void AtPadFFT::SetFFTData(TVirtualFFT *fft)
{
   assert(fft->GetN()[0] / 2 + 1 == fRe.size());
   fft->SetPointsComplex(fRe.data(), fIm.data());
}
void AtPadFFT::SetPoint(int i, TComplex val)
{
   assert(i < fRe.size());
   SetPointIm(i, val.Im());
   SetPointRe(i, val.Re());
}

std::unique_ptr<AtPadFFT> AtPadFFT::CreateFromFFT(const TVirtualFFT *fft)
{
   auto ret = std::make_unique<AtPadFFT>();
   ret->GetDataFromFFT(fft);
   return ret;
}

ClassImp(AtPadFFT);
