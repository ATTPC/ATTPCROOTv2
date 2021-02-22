#ifndef ATPSFILTER_H
#define ATPSFILTER_H

#include "ATPSA.hh"

// ROOT classes

class ATPSAFilter:public ATPSA {
 public:
    ATPSAFilter();
    ~ATPSAFilter();

    void Analyze(ATRawEvent * rawEvent, ATEvent * event) override;

    void SetBackGroundSuppression();
    void SetBackGroundInterpolation();
    void SetPeakFinder();
    void SetMaxFinder();
    void SetBaseCorrection(Bool_t value);
    void SetTimeCorrection(Bool_t value);

    void SetMeanK(Int_t value);	//Number of neighbors
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

 ClassDefOverride(ATPSAFilter, 1)};

#endif
