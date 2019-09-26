#include "ATPSAFull.hh"
#include "TH1F.h"
#include "TRotation.h"
#include "TMatrixD.h"
#include "TArrayD.h"

// STL
#include <cmath>
#include <map>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <algorithm>

ClassImp(ATPSAFull)

ATPSAFull::ATPSAFull(){
  //fPeakFinder = new TSpectrum();
}

ATPSAFull::~ATPSAFull(){
}

void
ATPSAFull::Analyze(ATRawEvent *rawEvent, ATEvent *event) {
  Int_t numPads = rawEvent -> GetNumPads();
  Int_t hitNum = 0;
  std::map<Int_t,Int_t> PadMultiplicity;
  Float_t mesh[512] = {0};
  
  for (auto iPad = 0; iPad < numPads; iPad++) {
    ATPad *pad = rawEvent -> GetPad(iPad);
    Int_t PadNum = pad -> GetPadNum();
    Int_t PadHitNum = 0;
    TVector3 HitPos;
    TVector3 HitPosRot;
    TRotation r;
    TRotation ry;
    TRotation rx;
    r.RotateZ(272.0*TMath::Pi()/180.0);
    ry.RotateY(180.0*TMath::Pi()/180.0);
    rx.RotateX(6.0*TMath::Pi()/180.0);
    
    Double_t xPos = pad -> GetPadXCoord();
    Double_t yPos = pad -> GetPadYCoord();
    Double_t zPos = 0;
    Double_t xPosRot = 0;
    Double_t yPosRot = 0;
    Double_t zPosRot = 0;
    Double_t xPosCorr = 0;
    Double_t yPosCorr = 0;
    Double_t zPosCorr = 0;
    Double_t charge = 0;
    Int_t maxAdcIdx=0;
    Int_t numPeaks=0;
    
    CalcLorentzVector();
    
    if (!(pad -> IsPedestalSubtracted())) {
      fLogger -> Error(MESSAGE_ORIGIN, "Pedestal should be subtracted to use this class!");
      //return;
    }
    
    Double_t *adc = pad -> GetADC();
    Double_t floatADC[512] = {0};
    Double_t dummy[512] = {0};
    
    Int_t divider = 50;
    Int_t initial=0;
    Int_t final=0;
    Int_t endInterval=0;
    Float_t max=0.0;
    Int_t maxTime = 0;
    
    std::map<Int_t, Int_t> interval;

    for (Int_t iTb = 0; iTb < fNumTbs; iTb++) floatADC[iTb] = adc[iTb];

      for (Int_t ij = 20; ij < 500; ij++) { //Excluding first and last 12 Time Buckets
        if (floatADC[ij] > max) {
  	     max = floatADC[ij];
  	     maxTime = ij;
        }
      }
    Int_t size=0;
    maxAdcIdx = maxTime;
    for (Int_t iTb = 0; iTb < fNumTbs; iTb++){
      mesh[iTb] += floatADC[iTb];
      if(floatADC[iTb]>fThreshold){
	if(iTb==0) initial = 0;
	else if(floatADC[iTb-1]<fThreshold) initial = iTb;
	if(iTb==(fNumTbs-1)) {
	  final = iTb;
	  if(final-initial>divider) size=final-initial;
	  interval.insert(std::pair<Int_t,Int_t>(initial,final));
	}
	else if(floatADC[iTb+1]<fThreshold){
	  final = iTb;
	  if(final-initial>divider) size=final-initial;
	  interval.insert(std::pair<Int_t,Int_t>(initial,final));
	}
      }
    }

    if(size<divider){
      double zPos   = CalculateZGeo(maxTime);
      double xPos   = pad -> GetPadXCoord();
      double yPos   = pad -> GetPadYCoord();
      if((xPos<-9000 || yPos<-9000) && pad->GetPadNum()!=-1) 
	std::cout << " ATPSAFull::Analysis Warning! Wrong Coordinates for Pad : "
		  << pad->GetPadNum()<<std::endl;
      ATHit *hit = new ATHit(PadNum,hitNum, xPos, yPos, zPos, max);
      hit->SetTimeStamp(maxTime);
      event -> AddHit(hit);
      delete hit;
      hitNum++;
      PadMultiplicity.insert(std::pair<Int_t,Int_t>(PadNum,hitNum));  
    }
    else {
      std::map<Int_t,Int_t>::iterator ite = interval.begin(); 
      //Double_t *thePar = new Double_t[3];
      while(ite!=interval.end()){
	final = (ite->second);
	initial = (ite->first);
	Int_t reducedPoints = (final-initial)/divider;
	for(Int_t points=0; points<reducedPoints+1; points++){
	  Int_t initInterval=initial+points*divider;
	  if(points==reducedPoints) endInterval=final;
	  else endInterval =  initial+((points+1)*divider)-1;
	  
	  double zPos   = CalculateZGeo(initInterval);
	  double xPos   = pad -> GetPadXCoord();
	  double yPos   = pad -> GetPadYCoord();
	  if((xPos<-9000 || yPos<-9000) && pad->GetPadNum()!=-1) 
	    std::cout << " ATPSAFull::Analysis Warning! Wrong Coordinates for Pad : "
		      << pad->GetPadNum()<<std::endl;
	  
	  for(Int_t iIn=initInterval;iIn<endInterval;iIn++) charge += floatADC[iIn]/divider;//reduced by divider!!!
	  ATHit *hit = new ATHit(PadNum,hitNum, xPos, yPos, zPos, charge);
	  hit->SetTimeStamp(initInterval);
	  charge=0;
	  
	  event -> AddHit(hit);
	  delete hit;
	  hitNum++;
	  
	  PadMultiplicity.insert(std::pair<Int_t,Int_t>(PadNum,hitNum));  
	}
	ite++;
      } 
    interval.clear();
    }
  }//Pad Loop
  
  for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
    event -> SetMeshSignal(iTb,mesh[iTb]);
  
  event->SortHitArrayTime();
  event -> SetMultiplicityMap(PadMultiplicity);

}
