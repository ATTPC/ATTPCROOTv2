/*******************************************************************
* Daughter class for Linear Hough Space transformation             *
* Log: Class started 28-04-2015                                    *
* Author: Y. Ayyad (NSCL ayyadlim@nscl.msu.edu)                    *
********************************************************************/

#ifndef ATHOUGHSPACELINE_H
#define ATHOUGHSPACELINE_H

#include "ATHoughSpace.hh"
#include "TH2F.h"


class ATHoughSpaceLine : public ATHoughSpace{

      public:
	       ATHoughSpaceLine();
        ~ATHoughSpaceLine();

	      TH2F* GetHoughSpace(TString ProjPlane);
        void CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane);
        void CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4);
        TH2F* GetHoughQuadrant(Int_t index);

      protected:
        Int_t fThreshold;
        std::map<std::vector<Float_t>,Int_t> HoughMap_XZ;
        std::vector<Float_t> HoughPar;
        TH2F *HistHoughXZ;
        TH2F *HistHoughRZ[4]; //One per quadrant


        ClassDef(ATHoughSpaceLine, 1);

};

#endif
