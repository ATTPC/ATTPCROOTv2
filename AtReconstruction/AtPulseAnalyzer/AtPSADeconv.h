#ifndef ATPSADECONV_H
#define ATPSADECONV_H

#include "AtPSA.h"
#include "AtPadFFT.h"

#include <TClonesArray.h>

#include <vector>

class TVirtualFFT;
class AtPadCharge;
class AtRawEvent;

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
   AtPad fResponse;

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

   void SetResponse(AtPad response);

   const AtPad &GetResponse() { return fResponse; }
   const AtPadFFT *GetResponseFFT();

protected:
   /// Data structure for Z loc (TB), Z sigma (TB), Charge (arb), Charge sigma (arb)
   using HitData = std::vector<std::array<double, 4>>;

   /**
    * Takes a pad with charge information and returns a list of hits to add to the event.
    */
   virtual HitVector chargeToHits(AtPad *charge);

   /**
    * Returns the salient data from the charge distribution:
    * Hit charge/location in Z and their std. deviation. Called by chargeToHits.
    * By default it returns the weighted average of the charge and it's variance.
    */
   virtual HitData getZandQ(const AtPad::trace &charge);
   virtual double getZhitVariance(double zLoc, double zLocVar) const override;

   void initFFTs();
   void initFilter();
   double getFilterKernel(int freq);

   /// Assumes that the pad has it's fourier transform information filled.
   HitVector AnalyzeFFTpad(AtPad *pad);
};

#endif //#ifndef ATPSADECONV_H
