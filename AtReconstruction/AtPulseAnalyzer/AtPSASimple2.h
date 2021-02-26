#ifndef AtPSASIMPLE2_H
#define AtPSASIMPLE2_H

#include "AtPSA.h"

class AtPSASimple2 : public AtPSA {
public:
   AtPSASimple2();
   ~AtPSASimple2();

   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;

   void SetBackGroundSuppression();
   void SetBackGroundInterpolation();
   void SetPeakFinder();
   void SetMaxFinder();
   void SetBaseCorrection(Bool_t value);
   void SetTimeCorrection(Bool_t value);

private:
   Bool_t fBackGroundSuppression;
   Bool_t fBackGroundInterp;
   Bool_t fIsPeakFinder;
   Bool_t fIsMaxFinder;
   Bool_t fIsBaseCorr;
   Bool_t fIsTimeCorr;

   ClassDefOverride(AtPSASimple2, 2)
};

#endif
