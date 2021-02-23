#include "AtAnalysis.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(AtAnalysis)

AtAnalysis::AtAnalysis()
{
  fLogger = FairLogger::GetLogger();

  FairRun *run = FairRun::Instance();
  if (!run)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No analysis run!");

  FairRuntimeDb *db = run -> GetRuntimeDb();
  if (!db)
    fLogger -> Fatal(MESSAGE_ORIGIN, "No runtime database!");

  fPar = (AtDigiPar *) db -> getContainer("AtDigiPar");
  if (!fPar)
    fLogger -> Fatal(MESSAGE_ORIGIN, "AtDigiPar not found!!");

    fNumTbs = fPar -> GetNumTbs();
    fTBTime = fPar -> GetTBTime();
    fDriftVelocity = fPar -> GetDriftVelocity();
    fMaxDriftLength = fPar -> GetDriftLength();
    fBField = fPar->GetBField();
    fEField = fPar->GetEField();
    fTiltAng = fPar->GetTiltAngle();
    fTB0  =  fPar->GetTB0();
    fZk   = fPar->GetZPadPlane();
    fEntTB   = (Int_t) fPar->GetTBEntrance();
    fThetaPad        = fPar->GetThetaPad()*TMath::Pi()/180.0;
    fThetaRot        = fPar->GetThetaRot()*TMath::Pi()/180.0;
    fThetaLorentz    = fPar->GetThetaLorentz()*TMath::Pi()/180.0;
    fPressure        = fPar->GetGasPressure();
    fMaxRange        = fPar->GetMaxRange();
    fCoefT           = fPar->GetCoefDiffusionTrans();
    fCoefL           = fPar->GetCoefDiffusionLong();
    fGain            = fPar->GetGain();



}

AtAnalysis::~AtAnalysis()
{
}
