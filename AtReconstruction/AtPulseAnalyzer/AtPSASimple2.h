#ifndef AtPSASIMPLE2_H
#define AtPSASIMPLE2_H

#include "AtPSA.h"

#include <Rtypes.h> // for Bool_t, THashConsistencyHolder, ClassDefOverride
class AtEvent;
class AtRawEvent;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPSASimple2 : public AtPSA {
public:
   AtPSASimple2() = default;
   ~AtPSASimple2() = default;

   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;

   void SetBackGroundSuppression();
   void SetBackGroundInterpolation();
   void SetPeakFinder();
   void SetMaxFinder();
   void SetBaseCorrection(Bool_t value);
   void SetTimeCorrection(Bool_t value);

private:
   Bool_t fBackGroundSuppression{false};
   Bool_t fBackGroundInterp{false};
   Bool_t fIsPeakFinder{false};
   Bool_t fIsMaxFinder{false};
   Bool_t fIsBaseCorr{false};
   Bool_t fIsTimeCorr{false};

   ClassDefOverride(AtPSASimple2, 2)
};

#endif
