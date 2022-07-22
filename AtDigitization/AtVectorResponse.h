#ifndef ATVECTORRESPONSE_H
#define ATVECTORRESPONSE_H

#include <Rtypes.h>
#include <TString.h>

#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

class AtVectorResponse {
protected:
   static Double_t fPeakingTime; //! Electronic peaking time in us
   static Double_t fTBTime;      //! Time bucket size in us
   static std::vector<Double_t> fWaveSample;

public:
   AtVectorResponse();
   static double ResponseFunction(double reducedTime);
   static void SetFile(TString filename);

protected:
   static void setParameters();
};
#endif
