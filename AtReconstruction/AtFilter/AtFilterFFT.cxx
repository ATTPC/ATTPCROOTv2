#include "AtFilterFFT.h"

#include "AtPad.h"
#include "AtPadFFT.h"
#include "AtRawEvent.h"

#include <FairLogger.h>

#include <Rtypes.h>
#include <TVirtualFFT.h>

#include <iostream>
#include <utility>

bool AtFilterFFT::AddFreqRange(AtFreqRange range)
{
   auto canAdd = isValidFreqRange(range);
   if (canAdd) {
      fFreqRanges.push_back(std::move(range));
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
   if (fSaveTransform) {
      replacePadWithPadFFT(inputEvent);
      fTransformedEvent = inputEvent;
   }
}

void AtFilterFFT::Filter(AtPad *pad)
{
   // Get data and transform
   if (!pad->IsPedestalSubtracted()) {
      LOG(error) << "Skipping FFT on pad " << pad->GetPadNum() << " at " << pad << " because not pedestal subtracted.";
      return;
   }

   fFFT->SetPoints(pad->GetADC().data());
   fFFT->Transform();
   applyFrequencyCutsAndSetInverseFFT();
   fFFTbackward->Transform();

   for (int i = 0; i < pad->GetADC().size(); ++i)
      pad->SetADC(i, fFFTbackward->GetPointReal(i));

   if (fSaveTransform) {
      auto inputPad = dynamic_cast<AtPadFFT *>(fTransformedEvent->GetPad(pad->GetPadNum()));
      inputPad->GetDataFromFFT(fFFT.get());
      auto outputPad = dynamic_cast<AtPadFFT *>(pad);
      outputPad->GetDataFromFFT(fFFTbackward.get());
   }
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

void AtFilterFFT::applyFrequencyCutsAndSetInverseFFT()
{
   for (int i = 0; i < fFFT->GetN()[0]; ++i) {
      Double_t re, im;
      fFFT->GetPointComplex(i, re, im);
      if (fFactors.find(i) != fFactors.end()) {
         re *= fFactors[i];
         im *= fFactors[i];
      }
      fFFTbackward->SetPoint(i, re / fFFT->GetN()[0], im / fFFT->GetN()[0]);
   }
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

void AtFilterFFT::replacePadWithPadFFT(AtRawEvent *event)
{
   for (auto &pad : event->fPadList)
      pad = std::make_unique<AtPadFFT>(*pad);
}
