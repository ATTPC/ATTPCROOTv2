// =================================================
//  GETMath Class
//
//  Author:
//    Genie Jhang ( geniejhang@majimak.com )
//
//  Log:
//    - 2013. 10. 24
//      Start writing class
//    - 2015. 11. 13
//      Adopted to the new version
// =================================================

#include <cmath>

#include "GETMath2.h"

ClassImp(GETMath2)

   GETMath2::GETMath2()
{
   Reset();
}

void GETMath2::Add(Double_t value)
{
   /**
    * This method uses the formulae below to calculate mean and RMS. <br>
    * \f$\text{mean}\Rightarrow \mu_{n+1} = \mu_n + \displaystyle\frac{x_{n+1} - \mu_n}{n + 1},\quad(n\geq0)\f$ <br>
    * \f$\text{RMS}\Rightarrow\sigma^2_{n+1} = \displaystyle\frac{n}{n + 1}\sigma^2_n + \displaystyle\frac{(x_{n+1} -
    *\mu_{n+1})^2}{n},\quad(n>0)\f$
    **/
   fMean += (value - fMean) / (Double_t)(fNumValues + 1);

   if (fNumValues > 0)
      fRms = fNumValues * fRms / (Double_t)(fNumValues + 1) + pow(value - fMean, 2) / (Double_t)(fNumValues);

   fNumValues++;
}

Double_t GETMath2::GetMean()
{
   return fMean;
}
Double_t GETMath2::GetRMS()
{
   return sqrt(fRms);
}
Double_t GETMath2::GetRMS2()
{
   return fRms;
}

void GETMath2::Set(Int_t numValues, Double_t mean, Double_t rms2)
{
   fNumValues = numValues;
   fMean = mean;
   fRms = rms2;
}

void GETMath2::Reset()
{
   fNumValues = 0;
   fMean = 0;
   fRms = 0;
}
