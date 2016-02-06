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

#include "TObject.h"
#include "ATEvent.hh"


class ATMinimization : public TObject
{

	public:

		ATMinimization();
		virtual ~ATMinimization();

    virtual Int_t GetMinimization()=0;
	  virtual Bool_t Minimize(Double_t* parameter,ATEvent *event)=0;
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

		struct FitPar
		{
			Double_t sThetaMin;
			Double_t sEnerMin;
			TVector3 sPosMin;
			Double_t sBrhoMin;
			Double_t sBMin;
			Double_t sPhiMin;
			Double_t sChi2Min;

		};

		FitPar FitParameters;


		ClassDef(ATMinimization, 1);

};

#endif
