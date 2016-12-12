/*******************************************************************
* Base class for Minimization                                      *
* Log: Class started 28-10-2015                                    *
* Author: Y. Ayyad (NSCL ayyadlim@nscl.msu.edu)                    *
********************************************************************/

#ifndef ATMINIMIZATION_H
#define ATMINIMIZATION_H

#include "FairRootManager.h"
#include "FairLogger.h"

//ROOT classes
#include "TClonesArray.h"
#include "TMath.h"
#include "TH2Poly.h"

#include "AtTpcMap.h"

#include "TObject.h"
#include "ATEvent.hh"

#ifndef __CINT__ // Boost
#include <boost/multi_array.hpp>
#endif //__CINT__


class ATMinimization : public TObject
{

	public:

		ATMinimization();
		virtual ~ATMinimization();

		typedef boost::multi_array<double,3> multiarray;
		typedef multiarray::index index;

    virtual Int_t GetMinimization()=0;
	  virtual Bool_t Minimize(Double_t* parameter,ATEvent *event)=0;
		virtual Bool_t MinimizeOpt(Double_t* parameter,ATEvent *event)=0;
		virtual Bool_t MinimizeOptMap(Double_t* parameter,ATEvent *event,TH2Poly* hPadPlane)=0;
		virtual Bool_t MinimizeOptMapAmp(Double_t* parameter,ATEvent *event, TH2Poly* hPadPlane,const multiarray& PadCoord)=0;
		virtual std::vector<ATHit> GetTBHitArray(Int_t TB,std::vector<ATHit> *harray)=0;
		virtual std::vector<Double_t> GetPosXMin()=0;
		virtual std::vector<Double_t> GetPosYMin()=0;
		virtual std::vector<Double_t> GetPosZMin()=0;
		virtual std::vector<Double_t> GetPosXExp()=0;
		virtual std::vector<Double_t> GetPosYExp()=0;
		virtual std::vector<Double_t> GetPosZExp()=0;
		virtual std::vector<Double_t> GetPosXInt()=0;
		virtual std::vector<Double_t> GetPosYInt()=0;
		virtual std::vector<Double_t> GetPosZInt()=0;
		virtual std::vector<Double_t> GetPosXBack()=0;
		virtual std::vector<Double_t> GetPosYBack()=0;
		virtual std::vector<Double_t> GetPosZBack()=0;
		virtual void ResetParameters()=0;

		struct FitPar
		{
			Double_t sThetaMin;
			Double_t sEnerMin;
			TVector3 sPosMin;
			Double_t sBrhoMin;
			Double_t sBMin;
			Double_t sPhiMin;
			Double_t sChi2Min;
			TVector3 sVertexPos;
			Double_t sVertexEner;
			Double_t sMinDistAppr;
			Int_t    sNumMCPoint;
			Double_t sNormChi2;
			Double_t sChi2Q;
			Double_t sChi2Range;


		};

		FitPar FitParameters;


		ClassDef(ATMinimization, 1);

};

#endif
