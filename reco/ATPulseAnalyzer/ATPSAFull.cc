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

    for (Int_t iTb = 0; iTb < fNumTbs; iTb++){
      floatADC[iTb] = adc[iTb];
      if(floatADC[iTb]>fThreshold){
	double zPos   = CalculateZGeo(iTb);
	double xPos   = pad -> GetPadXCoord();
	double yPos   = pad -> GetPadYCoord();
	
	if((xPos<-9000 || yPos<-9000) && pad->GetPadNum()!=-1) 
	  std::cout<<" ATPSASimple2::Analysis Warning! Wrong Coordinates for Pad : "<<pad->GetPadNum()<<std::endl;

	double charge = floatADC[iTb];
	ATHit *hit = new ATHit(PadNum,hitNum, xPos, yPos, zPos, charge);
	hit->SetTimeStamp(iTb);

	event -> AddHit(hit);
	delete hit;
	hitNum++;
	
	for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
	  mesh[iTb]+=floatADC[iTb];
	
	PadMultiplicity.insert(std::pair<Int_t,Int_t>(PadNum,hitNum));  
	
      }
    }
  }//Pad Loop
  
  for (Int_t iTb = 0; iTb < fNumTbs; iTb++)
    event -> SetMeshSignal(iTb,mesh[iTb]);
  
  event->SortHitArrayTime();
  event -> SetMultiplicityMap(PadMultiplicity);
}
