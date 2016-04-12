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
  void CalcHoughSpace(ATEvent* event,AtTpcMap* map);
  void CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4);
        //TH2F* GetHoughQuadrant(Int_t index);
	std::pair<Double_t,Double_t> GetHoughParameters(TH2F* hist);
  std::pair<Double_t,Double_t> GetHoughParameters(); //Overloaded version for std::map
	std::vector<std::pair<Double_t,Double_t>> GetHoughPar(TString opt="Hist");
  void FillHoughMap(Double_t ang, Double_t dist);
  void SetRadiusThreshold(Float_t value);


      protected:
        Int_t fThreshold;
        Int_t fRadThreshold;
        std::map<std::vector<Float_t>,Int_t> HoughMap_XZ;
        TH2F *HistHoughXZ;
        //TH2F *HistHoughRZ[4]; //One per quadrant
      	std::vector<std::pair<Double_t,Double_t>> HoughPar;
        std::vector<std::pair<Double_t,Double_t>> HoughParSTD;
        std::map<ULong64_t,Int_t> HoughMap; //8 byte for the key, unsigned, no negative distance in Linear Hough space is expected for each quadrant (Radius and Z are the vairbales)
        std::vector<ULong64_t> HoughMapKey;

        struct maxpersecond
        {
            template <typename Lhs, typename Rhs>
              bool operator()(const Lhs& lhs, const Rhs& rhs) const
                {
                  return lhs.second < rhs.second;
                }
        };


        ClassDef(ATHoughSpaceLine, 1);

};

#endif
