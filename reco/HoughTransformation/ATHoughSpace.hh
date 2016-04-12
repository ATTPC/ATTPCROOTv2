/*******************************************************************
* Base class for Hough Space transformation for the ATTPCROOT      *
* Log: Class started 28-04-2015                                    *
* Author: Y. Ayyad (NSCL ayyadlim@nscl.msu.edu)                    *
********************************************************************/


#ifndef ATHOUGHSPACE_H
#define ATHOUGHSPACE_H


#include "ATHit.hh"
#include "ATEvent.hh"
#include "ATProtoEvent.hh"
#include "ATProtoQuadrant.hh"
#include "ATDigiPar.hh"
#include "AtTpcMap.h"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

//ROOT classes
#include "TClonesArray.h"
#include "TH2F.h"
#include "TMath.h"

#include "TObject.h"

class ATHoughSpace : public TObject
{

	public:

		ATHoughSpace();
		virtual ~ATHoughSpace();

         virtual TH2F* GetHoughSpace(TString ProjPlane)=0;
	       virtual void CalcHoughSpace(ATEvent* event,Bool_t YZplane,Bool_t XYplane, Bool_t XZplane)=0;
				 virtual void CalcHoughSpace(ATEvent* event,AtTpcMap* map)=0;
				 virtual void CalcHoughSpace(ATProtoEvent* protoevent,Bool_t q1,Bool_t q2, Bool_t q3, Bool_t q4)=0;

				 void SetThreshold(Double_t value);

   protected:


		  Double_t fThreshold;


		ClassDef(ATHoughSpace, 2);

};

#endif
