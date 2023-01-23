#ifndef ATPSADECONV_H
#define ATPSADECONV_H

#include "AtPSA.h"
#include "AtPad.h"
#include "AtRawEvent.h"

#include <TVirtualFFT.h> // for TVirtualFFT

#include <array> // for array
#include <cmath>
#include <functional>
#include <memory> // for unique_ptr, make_unique
#include <string> // for string
#include <utility>
#include <vector>

class AtPadFFT;

/**
 * @brief Abstract base class for getting current through deconvolution.
 *
 * This PSA method calculates the charge collected over a pad. It operates as a base class
 * where specific PSA methods can be implemented on top of this class to convert the current
 * into hits in 3D space.
 *
 * Mathmatically what this class does is take in an electronic response function r(TB), and stores
 * the FFT of that response function R(k). For the trace recorded in each pad s(TB), we then calculate
 * the FFT of that trace S(k), and divide by R(k). We then apply a low-pass Buttersworth filter of
 * even order n with cuttoff frequency w_c (both specified by the user). This, transformed back into
 * the time domain, should give the input current over the pad.
 *
 * It saves the reconstruced charge as an augment (AtPadArray with name "Qreco") to the
 * pad in the input event
 *
 * @ingroup PSA
 * @author A.K. Anthony
 */
class AtPSADeconv : public virtual AtPSA {

protected:
   using ResponseFunc = std::function<double(int, double)>;

   /**
    * AtRawEvent holding the response function and filter for every pad. If there is only a single
    * response being used for the entire detector, than always use pad 0.
    *
    * Each pad in this event must have two augments. An AtPadFFT of the name "fft" which holds the
    * FFT of the response, and an AtPadFFT with the name "filter" which holds the FFT of the response
    * multiplied by the low pass filter in use. When a pad is accessed, it will check for the existance
    * of these two augments and add them if necessary.
    */
   AtRawEvent fEventResponse;
   /**
    * Callable object representing the response function of the detecotor. Will be used to fill
    * fEventResponse if the pad does not already exist within that event.
    */
   ResponseFunc fResponse{nullptr};

   std::unique_ptr<TVirtualFFT> fFFT{nullptr};
   std::unique_ptr<TVirtualFFT> fFFTbackward{nullptr};

   int fFilterOrder{0}; //< Half the filter order
   int fCutoffFreq{-1}; //< Cutoff frequency squared

public:
   AtPSADeconv();
   AtPSADeconv(AtPSADeconv &&obj) = default;
   AtPSADeconv &operator=(AtPSADeconv &&obj) = default;
   AtPSADeconv(const AtPSADeconv &obj);
   ~AtPSADeconv() = default;

   virtual std::unique_ptr<AtPSA> Clone() override { return std::make_unique<AtPSADeconv>(*this); }
   virtual void Init() override;
   virtual HitVector AnalyzePad(AtPad *pad) override;

   void SetFilterOrder(int order);
   void SetCutoffFreq(int freq);

   int GetFilterOrder() { return fFilterOrder * 2; }
   int GetCutoffFreq() { return sqrt(fCutoffFreq); }

   /**
    * Copy an AtRawEvent to use as the response function. If the pad number requested does
    * not exist in the AtRawEvent it will use the callable object stored in fResponse.
    */
   void SetResponse(AtRawEvent response) { fEventResponse = std::move(response); }
   /**
    * Response function to use if the AtRawEvent representation of the response function does not
    * contain the pad we are looking for. When this is used to get the response function, it is cached
    * in the internal fEventResponse.
    */
   void SetResponse(ResponseFunc response) { fResponse = response; }

   AtPad &GetResponse(int padNum);
   const AtPadFFT &GetResponseFFT(int padNum);
   const AtPadFFT &GetResponseFilter(int padNum);

protected:
   /// Data structure for Z loc (TB), Z sigma (TB), Charge (arb), Charge sigma (arb)
   using HitData = std::vector<std::array<double, 4>>;

   AtPad *createResponsePad(int padNum);

   /**
    * Takes a pad with charge information and returns a list of hits to add to the event.
    */
   virtual HitVector chargeToHits(AtPad &charge, std::string qName);

   /**
    * Returns the salient data from the charge distribution:
    * Hit charge/location in Z and their std. deviation. Called by chargeToHits.
    * By default it returns the weighted average of the charge and it's variance.
    */
   virtual HitData getZandQ(const AtPad::trace &charge);
   virtual double getZhitVariance(double zLoc, double zLocVar) const override;

   void initFFTs();
   void initFilter();
   void updateFilter(const AtPadFFT &fft, AtPadFFT *filter);
   double getFilterKernel(int freq);

   /// Assumes that the pad has it's fourier transform information filled.
   HitVector AnalyzeFFTpad(AtPad &pad);
};

#endif //#ifndef ATPSADECONV_H
