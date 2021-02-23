#ifndef AtPSFILTER_H
#define AtPSFILTER_H

#include "AtPSA.h"

// ROOT classes

class AtPSAFilter : public AtPSA {
public:
   AtPSAFilter();
   ~AtPSAFilter();

   void Analyze(AtRawEvent *rawEvent, AtEvent *event) override;

   void SetBackGroundSuppression();
   void SetBackGroundInterpolation();
   void SetPeakFinder();
   void SetMaxFinder();
   void SetBaseCorrection(Bool_t value);
   void SetTimeCorrection(Bool_t value);

   void SetMeanK(Int_t value); // Number of neighbors
   void SetStddevMulThresh(Double_t value);

private:
   Bool_t fBackGroundSuppression;
   Bool_t fBackGroundInterp;
   Bool_t fIsPeakFinder;
   Bool_t fIsMaxFinder;
   Bool_t fIsBaseCorr;
   Bool_t fIsTimeCorr;

   Int_t fMeanK;
   Double_t fStdDev;

   ClassDefOverride(AtPSAFilter, 1)
};

#endif
