#ifndef AtPSASIMPLE2_H
#define AtPSASIMPLE2_H

#include "AtCalibration.h" // for AtCalibration
#include "AtPSA.h"

#include <Rtypes.h>  // for Bool_t, THashConsistencyHolder, ClassDefOverride
#include <TString.h> // for TString

#include <memory> // for make_unique, unique_ptr

class AtEvent;
class AtPad;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

class [[deprecated("Use AtPSASpectrum or AtPSAMax instead")]] AtPSASimple2 : public AtPSA
{
private:
   AtCalibration fCalibration;

   Bool_t fBackGroundSuppression{false};
   Bool_t fBackGroundInterp{false};
   Bool_t fIsPeakFinder{false};
   Bool_t fIsMaxFinder{false};
   Bool_t fIsBaseCorr{false};
   Bool_t fIsTimeCorr{false};

public:
   void Analyze(AtRawEvent * rawEvent, AtEvent * event) override;
   HitVector AnalyzePad(AtPad * pad) override
   {
      return {};
   };
   std::unique_ptr<AtPSA> Clone() override
   {
      return std::make_unique<AtPSASimple2>(*this);
   }

   void SetGainCalibration(TString gainFile)
   {
      fCalibration.SetGainFile(gainFile);
   }
   void SetJitterCalibration(TString jitterFile)
   {
      fCalibration.SetJitterFile(jitterFile);
   }
   void SetBackGroundSuppression();
   void SetBackGroundInterpolation();
   void SetPeakFinder();
   void SetMaxFinder();
   void SetBaseCorrection(Bool_t value);
   void SetTimeCorrection(Bool_t value);

   ClassDefOverride(AtPSASimple2, 2)
};

#endif
