#include "ATHit.hh"

ATHit::ATHit()
{
  fTrackID = -1;
  SetHit(-1, 0, 0, -1000, -1);

  fIsClustered = kFALSE;
  fClusterID = -1;
}

ATHit::ATHit(Int_t hitID, TVector3 vec, Double_t charge)
{
  fTrackID = -1;
  SetHit(hitID, vec, charge);

  fIsClustered = kFALSE;
  fClusterID = -1;
}

ATHit::ATHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge)
{
  fTrackID = -1;
  SetHit(hitID, x, y, z, charge);

  fIsClustered = kFALSE;
  fClusterID = -1;
}

ATHit::ATHit(ATHit *hit)
{
  fTrackID = hit -> GetTrackID();
  SetHit(hit -> GetHitID(), hit -> GetPosition(), hit -> GetCharge());

  fIsClustered = hit -> IsClustered();
  fClusterID = hit -> GetClusterID();
}

ATHit::~ATHit()
{}

void ATHit::SetTrackID(Int_t trackID)                                   { fTrackID = trackID; }
void ATHit::SetHitID(Int_t hitID)                                       { fHitID = hitID; }
void ATHit::SetHit(Int_t hitID, TVector3 vec, Double_t charge)                       { fHitID = hitID; fPosition = vec; fCharge = charge; }
void ATHit::SetHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge) { fHitID = hitID; fPosition = TVector3(x, y, z); fCharge = charge; }

void ATHit::SetPosition(TVector3 vec)                                   { fPosition = vec; }
void ATHit::SetPosition(Double_t x, Double_t y, Double_t z)             { fPosition = TVector3(x, y, z); }
void ATHit::SetPosSigma(TVector3 vec)                                   { fPositionSigma = vec; }
void ATHit::SetPosSigma(Double_t dx, Double_t dy, Double_t dz)          { fPositionSigma = TVector3(dx, dy, dz); }
void ATHit::SetCharge(Double_t charge)                                  { fCharge = charge; }

void ATHit::SetIsClustered(Bool_t value)                                { fIsClustered = value; }
void ATHit::SetClusterID(Int_t clusterID)                               { fClusterID = clusterID; fIsClustered = kTRUE; }

Int_t ATHit::GetTrackID()                                               { return fTrackID; }
Int_t ATHit::GetHitID()                                                 { return fHitID; }
TVector3 ATHit::GetPosition()                                           { return fPosition; }
TVector3 ATHit::GetPosSigma()                                           { return fPositionSigma; }
Double_t ATHit::GetCharge()                                             { return fCharge; }
Bool_t ATHit::IsClustered()                                             { return fIsClustered; }
Int_t ATHit::GetClusterID()                                             { return (fIsClustered ? fClusterID : -1); }
