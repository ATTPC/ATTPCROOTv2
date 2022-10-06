#include "AtFilterFFT.h"

#include "AtPad.h"
#include "AtPadBase.h"
#include "AtPadFFT.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <Rtypes.h>
#include <TComplex.h> // IWYU pragma: keep
#include <TVirtualFFT.h>

#include <iostream>
#include <utility>

void AtFilterFFT::SetLowPass(int order, int cutoff)
{
   fFreqRanges.clear();
   for (int i = 0; i < 512 / 2 + 1; ++i)
      AddFreqRange({i, getFilterKernel(i, order, cutoff), i, getFilterKernel(i, order, cutoff)});
}

/**
 * @param[in] freq The frequency compnent
 * @return The kernel of the low pass filter as set at that frequency
 * @todo: Precompute once and save with the transformed response function
 */
double AtFilterFFT::getFilterKernel(int freq, int fFilterOrder, int fCutoffFreq)
{

   if (fFilterOrder == 0)
      return 1;

   return 1.0 / (1.0 + std::pow(freq * freq / fCutoffFreq, fFilterOrder));
}

bool AtFilterFFT::AddFreqRange(AtFreqRange range)
{
   auto canAdd = isValidFreqRange(range);
   if (canAdd) {
      fFreqRanges.push_back(std::move(range));

      if (range.fEndFreq == range.fBeginFreq) {
         fFactors[range.fBeginFreq] = range.fBeginFact;
         return true;
      }
      auto dFact = (range.fEndFact - range.fBeginFact) / (range.fEndFreq - range.fBeginFreq);
      for (int i = range.fBeginFreq; i <= range.fEndFreq; ++i)
         fFactors[i] = range.fBeginFact + dFact * (i - range.fBeginFreq);
   }
   return canAdd;
}

void AtFilterFFT::Init()
{
   std::vector<Int_t> dimSize = {fTransformSize};

   // Create a FFT object that we own ("K"), that will optimize the transform ("M"),
   // and is a forward transform from real data to complex ("R2C")
   fFFT = std::unique_ptr<TVirtualFFT>(TVirtualFFT::FFT(1, dimSize.data(), "R2C M K"));

   // Create a FFT object that we own ("K"), that will optimize the transform ("M"),
   // and is a backwards transform from complex to Reak ("C2R")
   fFFTbackward = std::unique_ptr<TVirtualFFT>(TVirtualFFT::FFT(1, dimSize.data(), "C2R M K"));
}

void AtFilterFFT::InitEvent(AtRawEvent *inputEvent)
{
   fInputEvent = inputEvent;
}

/**
 * If save transform is set, will replace the pad in the input AtRawEvent with an AtPadFFT that stores the
 * result of the FFT before any filtering. The output event will also contain AtPadFFTs with the transformed
 * waveforms after the filter on magnitude has been applied. Will multiply each complex fourier component by
 * a number defined in the frequency ranges, and store the resultant waveform in the filtered raw event.
 *
 */
void AtFilterFFT::Filter(AtPad *pad)
{
   // Get data and transform
   if (!pad->IsPedestalSubtracted()) {
      LOG(error) << "Skipping FFT on pad " << pad->GetPadNum() << " at " << pad << " because not pedestal subtracted.";
      return;
   }

   fFFT->SetPoints(pad->GetADC().data());
   fFFT->Transform();

   auto fft = applyFrequencyCutsAndSetInverseFFT();

   // If we are saving the transform add
   if (fSaveTransform) {
      // Add the frequency information to the output pad
      pad->AddAugment("fft", std::move(fft));

      // Add the freq information to the input pad
      auto inputFFT = std::make_unique<AtPadFFT>();
      inputFFT->GetDataFromFFT(fFFT.get());
      fInputEvent->GetPad(pad->GetPadNum())->AddAugment("fft", std::move(inputFFT));
   }

   fFFTbackward->Transform();

   double baseline = 0;
   if (fSubtractBackground) {
      for (int i = 0; i < 20; ++i)
         baseline += fFFTbackward->GetPointReal(i);
      baseline /= 20;
   }

   for (int i = 0; i < pad->GetADC().size(); ++i)
      pad->SetADC(i, fFFTbackward->GetPointReal(i) - baseline);

   // If we are saving the transform, add the
}

bool AtFilterFFT::isValidFreqRange(const AtFreqRange &range)
{
   bool isValid = true;
   isValid &= range.fBeginFreq <= range.fEndFreq;
   isValid &= range.fEndFreq <= fTransformSize / 2 + 1;
   isValid &= range.fBeginFact >= 0 && range.fBeginFact <= 1;
   isValid &= range.fEndFact >= 0 && range.fEndFact <= 1;
   isValid &= !doesFreqRangeOverlap(range);
   return isValid;
}

bool AtFilterFFT::doesFreqRangeOverlap(const AtFreqRange &newRange)
{
   for (auto &range : fFreqRanges) {
      // Check to make sure the minimum frequency does not exist within the bounds
      // of any added frequency, except for the max frequency when the factors are
      // the same
      if (newRange.fBeginFreq >= range.fBeginFreq && newRange.fBeginFreq < range.fEndFreq)
         return true;
      if (newRange.fBeginFreq == range.fEndFreq && newRange.fBeginFact != range.fEndFact)
         return true;

      // Check to make sure the maximum frequency does not exist within the bounds of any added
      // frequency, except for when it matches the minimum frequency with the same factor
      if (newRange.fEndFreq > range.fBeginFreq && newRange.fEndFreq <= range.fEndFreq)
         return true;
      if (newRange.fEndFreq == range.fBeginFreq && newRange.fEndFact != range.fBeginFreq)
         return true;
   }

   return false;
}

/**
 * Sets the C2R (inverse) FFT after applying the frequency cuts and returns a AtPadFFT object
 * that contains the frequency information used to set the C2R fft.
 */
std::unique_ptr<AtPadFFT> AtFilterFFT::applyFrequencyCutsAndSetInverseFFT()
{
   auto ret = std::make_unique<AtPadFFT>();
   for (int i = 0; i < 512 / 2 + 1; ++i) {

      Double_t re, im;
      fFFT->GetPointComplex(i, re, im);
      if (fFactors.find(i) != fFactors.end()) {
         re *= fFactors[i];
         im *= fFactors[i];
      }
      ret->SetPoint(i, {re, im});
      fFFTbackward->SetPoint(i, re / fFFT->GetN()[0], im / fFFT->GetN()[0]);
   }
   return ret;
}

bool operator<(const AtFilterFFT::AtFreqRange &lhs, const AtFilterFFT::AtFreqRange &rhs)
{
   return lhs.fBeginFreq < rhs.fBeginFreq;
}

void AtFilterFFT::DumpFactors()
{
   for (const auto &pair : fFactors)
      std::cout << pair.first << " " << pair.second << std::endl;
}
