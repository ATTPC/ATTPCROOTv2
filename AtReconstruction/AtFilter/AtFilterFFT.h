#ifndef ATFFTFILTER_H
#define ATFFTFILTER_H

#include "AtFilter.h"

#include <Rtypes.h>
#include <TVirtualFFT.h> // Annoyingly required for ROOT to generate a dictionary (even without IO)

#include <map>
#include <memory>
#include <vector>

class AtPad;
class AtPadFFT;
class AtRawEvent;
struct AtPadReference;

/**
 *  Filter for applying a FFT to data, and multiplying the transformed data by some factor. The factor
 *  is determined by a series of FreqRanges that define a start and stop frequency and associated
 *  factor at the start/stop. It then linearly interpelates the factor between the two frequencies.
 *  By default the factor is 1 (i.e. no adjustment to the transformed data). For alternative behavior,
 *  extend the class and override the protected function applyFreqCuts().
 *
 *  The output of this filter is an AtRawEvent where the fPadList is filled with pads of the type
 *  AtPadFFT so the fourier-space representation of the filtered data is saved.
 *  If you wish to save the unfiltered data with the fourier-space represetation, the set the flag
 *  fSaveTransform and the input branch will be modified to contain AtPadFFTs.
 *
 *  Adam Anthony 4/20/22
 * @ingroup RawFilters
 */
class AtFilterFFT : public AtFilter {

   // Forward declares and alias declerations
public:
   struct AtFreqRange {
      Int_t fBeginFreq;
      Double_t fBeginFact;

      Int_t fEndFreq;
      Double_t fEndFact;
      friend bool operator<(const AtFreqRange &lhs, const AtFreqRange &rhs);
   };
   using FreqRanges = std::vector<AtFreqRange>;

protected:
   FreqRanges fFreqRanges;
   std::map<Int_t, Double_t> fFactors;

   std::unique_ptr<TVirtualFFT> fFFT{nullptr};
   std::unique_ptr<TVirtualFFT> fFFTbackward{nullptr};

   Bool_t fSaveTransform{false};
   Bool_t fSubtractBackground{true};

   AtRawEvent *fInputEvent{nullptr};
   // AtRawEvent *fFilteredEvent{nullptr};
   static constexpr Int_t fTransformSize = 512;

public:
   AtFilterFFT() = default;
   ~AtFilterFFT() = default;

   bool AddFreqRange(AtFreqRange range); // Range is inclusive
   void SetSaveTransform(bool saveTransform) { fSaveTransform = saveTransform; }
   void SetSubtractBackground(bool subtractBackground) { fSubtractBackground = subtractBackground; }

   bool GetSaveTransform() { return fSaveTransform; }
   bool GetSubtractBackground() { return fSubtractBackground; }

   const FreqRanges &GetFreqRanges() { return fFreqRanges; }
   void DumpFactors();

   void Init() override;
   void InitEvent(AtRawEvent *event = nullptr) override;
   void Filter(AtPad *pad, AtPadReference *padReference) override;
   bool IsGoodEvent() override { return true; }
   void SetLowPass(int order, int cuttoff);

protected:
   virtual std::unique_ptr<AtPadFFT> applyFrequencyCutsAndSetInverseFFT();

private:
   bool isValidFreqRange(const AtFreqRange &range);
   bool doesFreqRangeOverlap(const AtFreqRange &range);
   double getFilterKernel(int freq, int fFilterOrder, int fCutoffFreq);
};

#endif //#ifndef ATFFTFILTER_H
