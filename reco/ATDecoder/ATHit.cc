#include "ATHit.hh"

ClassImp(ATHit)

ATHit::ATHit()
{
  fTrackID = -1;
  SetHit(-1, 0, 0, -1000, -1);

  fIsClustered = kFALSE;
  fClusterID = -1;
    fPadNum = -1;
    fQhit = -100.0;
    fHitMult = 0;
    fTimeStamp = 0;
    fBaseCorr = 0.0;
}

ATHit::ATHit(Int_t hitID, TVector3 vec, Double_t charge)
{
  fTrackID = -1;
  SetHit(hitID, vec, charge);

  fIsClustered = kFALSE;
  fClusterID = -1;
    fPadNum = -1;
    fQhit = -100.0;
    fHitMult = 0;
    fTimeStamp = 0;
    fBaseCorr = 0.0;
}

ATHit::ATHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge)
{
  fTrackID = -1;
  SetHit(hitID, x, y, z, charge);

  fIsClustered = kFALSE;
  fClusterID = -1;
    fPadNum = -1;
    fQhit = -100.0;
    fHitMult = 0;
    fTimeStamp = 0;
    fBaseCorr = 0.0;
}

ATHit::ATHit(Int_t PadNum,Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge)
{
    fTrackID = -1;
    SetHit(PadNum,hitID, x, y, z, charge);

    fIsClustered = kFALSE;
    fClusterID = -1;
    fQhit = -100.0;
    fHitMult = 0;
    fTimeStamp = 0;
    fBaseCorr = 0.0;

}

ATHit::ATHit(ATHit *hit)
{
  fTrackID = hit -> GetTrackID();
  SetHit(hit -> GetHitID(), hit -> GetPosition(), hit -> GetCharge());
  SetTimeStamp(hit->GetTimeStamp());
  fIsClustered = hit -> IsClustered();
  fClusterID = hit -> GetClusterID();
  fQhit = -100.0;
  fHitMult = 0;
  fBaseCorr = 0.0;
}

ATHit::~ATHit()
{}

void ATHit::SetTrackID(Int_t trackID)                                                               { fTrackID = trackID; }
void ATHit::SetHitID(Int_t hitID)                                                                   { fHitID = hitID; }
void ATHit::SetHit(Int_t hitID, TVector3 vec, Double_t charge)                                      { fHitID = hitID; fPosition = vec; fCharge = charge; }
void ATHit::SetHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge)                { fHitID = hitID; fPosition = TVector3(x, y, z); fCharge = charge; }
void ATHit::SetHit(Int_t PadNum,Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge)   { fPadNum= PadNum ; fHitID = hitID; fPosition = TVector3(x, y, z); fCharge = charge; }

void ATHit::SetPosition(TVector3 vec)                                   { fPosition = vec; }
void ATHit::SetPosition(Double_t x, Double_t y, Double_t z)             { fPosition = TVector3(x, y, z); }
void ATHit::SetPositionCorr(Double_t x, Double_t y, Double_t z)         { fPositionCorr = TVector3(x, y, z); }
void ATHit::SetPosSigma(TVector3 vec)                                   { fPositionSigma = vec; }
void ATHit::SetPosSigma(Double_t dx, Double_t dy, Double_t dz)          { fPositionSigma = TVector3(dx, dy, dz); }
void ATHit::SetCharge(Double_t charge)                                  { fCharge = charge; }

void ATHit::SetQHit(Double_t Qhit)                                      { fQhit = Qhit;}
void ATHit::SetHitMult(Int_t HitMult)				                           	{ fHitMult = HitMult;}
void ATHit::SetTimeStamp(Int_t Time)				                           	{ fTimeStamp = Time;}
void ATHit::SetBaseCorr(Double_t BaseCorr)                              { fBaseCorr = BaseCorr;}

void ATHit::SetIsClustered(Bool_t value)                                { fIsClustered = value; }
void ATHit::SetClusterID(Int_t clusterID)                               { fClusterID = clusterID; fIsClustered = kTRUE; }

Int_t ATHit::GetTrackID()                                               { return fTrackID; }
Int_t ATHit::GetHitID() const                                           { return fHitID; }
Int_t ATHit::GetHitPadNum()                                             { return fPadNum; }
TVector3 ATHit::GetPosition()                                           { return fPosition; }
TVector3 ATHit::GetPositionCorr()                                           { return fPositionCorr; }
TVector3 ATHit::GetPosSigma()                                           { return fPositionSigma; }
Double_t ATHit::GetCharge()                                             { return fCharge; }
Double_t ATHit::GetQHit()                                               { return fQhit;}
Int_t ATHit::GetHitMult()						                                    { return fHitMult;}
Int_t ATHit::GetTimeStamp()						                                  { return fTimeStamp;}
Bool_t ATHit::IsClustered()                                             { return fIsClustered; }
Int_t ATHit::GetClusterID()                                             { return (fIsClustered ? fClusterID : -1); }
Double_t ATHit::GetBaseCorr()                                           { return fBaseCorr; }
