#ifndef ATPSASIMPLE2_H
#define ATPSASIMPLE2_H

#include "ATPSA.hh"

class ATPSASimple2:public ATPSA {
 public:
    ATPSASimple2();
    ~ATPSASimple2();

    void Analyze(ATRawEvent * rawEvent, ATEvent * event) override;

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

 ClassDefOverride(ATPSASimple2, 2)};

#endif
