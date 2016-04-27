#include "ATPSA.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

// STL
#include <algorithm>

using std::max_element;
using std::min_element;
using std::distance;

ClassImp(ATPSA)

ATPSA::ATPSA()
{
  fLogger = FairLogger::GetLogger();

  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (ATDigiPar *) db -> getContainer("ATDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "ATDigiPar not found!!");

  fPadPlaneX = fPar -> GetPadPlaneX();
  fPadSizeX = fPar -> GetPadSizeX();
  fPadSizeZ = fPar -> GetPadSizeZ();
  fPadRows = fPar -> GetPadRows();
  fPadLayers = fPar -> GetPadLayers();

  fNumTbs = fPar -> GetNumTbs();
  fTBTime = fPar -> GetTBTime();
  fDriftVelocity = fPar -> GetDriftVelocity();
  fMaxDriftLength = fPar -> GetDriftLength();

  fBackGroundSuppression = kFALSE;
  fIsPeakFinder = kFALSE;
  fIsMaxFinder = kFALSE;
  fIsBaseCorr = kFALSE;
  fIsTimeCorr = kFALSE;

  fBField = fPar->GetBField();
  fEField = fPar->GetEField();
  fTiltAng = fPar->GetTiltAngle();
  fTB0  =  fPar->GetTB0();

  fEntTB   = (Int_t) fPar->GetTBEntrance();

  fThreshold = -1;


  std::cout<<" ==== Parameters for Pulse Shape Analysis Task ==== "<<std::endl;
  std::cout<<" ==== Magnetic Field : "<<fBField<<" T "<<std::endl;
  std::cout<<" ==== Electric Field : "<<fEField<<" V/cm "<<std::endl;
  std::cout<<" ==== Sampling Rate : "<<fTBTime<<" ns "<<std::endl;
  std::cout<<" ==== Tilting Angle : "<<fTiltAng<<" deg "<<std::endl;
  std::cout<<" ==== Drift Velocity : "<<fDriftVelocity<<" cm/us "<<std::endl;
  std::cout<<" ==== TB0 : "<<fTB0<<std::endl;

}

ATPSA::~ATPSA()
{
}

void ATPSA::SetBaseCorrection(Bool_t value)   { fIsBaseCorr=value;}
void ATPSA::SetTimeCorrection(Bool_t value)   { fIsTimeCorr=value;}

void
ATPSA::SetThreshold(Int_t threshold)
{
  fThreshold = threshold;
}

Double_t
ATPSA::CalculateX(Double_t row)
{
  return (row + 0.5)*fPadSizeX - fPadPlaneX/2.;
}

Double_t
ATPSA::CalculateZ(Double_t peakIdx)
{
  //return -peakIdx*fTBTime*fDriftVelocity/100.;
    //std::cout<<fNumTbs<<"    "<<fTBTime<<"	"<<fDriftVelocity<<" peakID : "<<peakIdx<<std::endl;
    return (fNumTbs - peakIdx)*fTBTime*fDriftVelocity/100.;
}

Double_t
ATPSA::CalculateZGeo(Double_t peakIdx)
{

    return fZk - (fEntTB - peakIdx)*fTBTime*fDriftVelocity/100.;
    // fZk defined in each daughter class

}

Double_t
ATPSA::CalculateY(Double_t layer)
{
  return (layer + 0.5)*fPadSizeZ;
}

void
ATPSA::SetBackGroundSuppression(){
  fBackGroundSuppression = kTRUE;
}

void
ATPSA::SetPeakFinder(){
  fIsPeakFinder= kTRUE;

}

void
ATPSA::SetMaxFinder(){
  fIsMaxFinder= kTRUE;

}

Double_t
ATPSA::CalculateXCorr(Double_t xvalue,Int_t Tbx)  //TODO: Yes, this can be done with one stupid function but try walking on my shoes...
{


  Double_t xcorr = xvalue - fLorentzVector.X()*(Tbx-fTB0)*fTBTime*1E-2; // Convert from ns to us and cm to mm
  return xcorr;


}
Double_t
ATPSA::CalculateYCorr(Double_t yvalue,Int_t Tby)
{
  Double_t ycorr = yvalue + fLorentzVector.Y()*(Tby-fTB0)*fTBTime*1E-2;
  return ycorr;
}
Double_t
ATPSA::CalculateZCorr(Double_t zvalue,Int_t Tbz)
{
  Double_t zcorr = fLorentzVector.Z()*(Tbz-fTB0)*fTBTime*1E-2;
  return zcorr;
}

void
ATPSA::CalcLorentzVector(){

       //fDriftVelocity*=-1;// TODO: Check sign of the Vd
       Double_t ot = (fBField/fEField)*fDriftVelocity*1E4;
       Double_t front = fDriftVelocity / (1 + ot*ot);
       Double_t TiltRad = fTiltAng*TMath::Pi()/180.0;

       Double_t x = front*ot*TMath::Sin(TiltRad);
       Double_t y = front*ot*ot*TMath::Cos(TiltRad)*TMath::Sin(TiltRad);
       Double_t z = front*(1+ ot*ot*TMath::Cos(TiltRad)*TMath::Cos(TiltRad));

       fLorentzVector.SetXYZ(x,y,z);

      //  std::cout<<" Vdx : "<<x<<std::endl;
      //   std::cout<<" Vdy : "<<y<<std::endl;
      //  std::cout<<" Vdz : "<<z<<std::endl;



}
