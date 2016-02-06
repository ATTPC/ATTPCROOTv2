/*******************************************************************
* Daughter class for Circular Hough Space transformation           *
* Log: Class started 26-10-2015                                    *
* Author: Y. Ayyad (NSCL ayyadlim@nscl.msu.edu)                    *
********************************************************************/

#ifndef ATHOUGHSPACECIRCLE_H
#define ATHOUGHSPACECIRCLE_H

#include "ATHoughSpace.hh"
#include "TH2F.h"


class ATHoughSpaceCircle : public ATHoughSpace{

      public:
	       ATHoughSpaceCircle();
        ~ATHoughSpaceCircle();

      	TH2F* GetHoughSpace(TString ProjPlane);
        void CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane);
        void CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4);
        Double_t GetXCenter() {return fXCenter;}
        Double_t GetYCenter() {return fYCenter;}
        std::vector<Double_t>* GetRadiusDist() {return fRadius;}
        std::vector<Int_t>* GetTimeStamp()  {return fTimeStamp;}
        std::vector<Double_t>* GetPhi()  {return fPhi;}
        std::vector<Double_t>* GetTheta()  {return fTheta;}


      protected:
        Int_t fThreshold;
        std::map<std::vector<Float_t>,Int_t> HoughMap_XY;
        std::vector<Float_t> HoughPar;
        std::vector<Double_t> *fRadius;
        std::vector<Int_t> *fTimeStamp;
        std::vector<Double_t> *fPhi;
        std::vector<Double_t> *fTheta;
        TH2F *HistHoughXY;
        TH2F *HistHoughAux;
        Double_t fXCenter;
        Double_t fYCenter;


        ClassDef(ATHoughSpaceCircle, 2);

};

#endif
