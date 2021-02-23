#include "AtHit.h"

ClassImp(AtHit)

AtHit::AtHit()
{
  fTrackID = -1;
  SetHit(-1, 0, 0, -1000, -1);

    fIsClustered = kFALSE;
    fClusterID = -1;
    fPadNum = -1;
    fQhit = -100.0;
    fHitMult = 0;
    fTimeStamp = 0;
    fTimeStampCorr = 0.0;
    fTimeStampCorrInter = 0.0;
    fBaseCorr = 0.0;
    fSlopeCnt=0;
    kIsAux = kFALSE;
    fMCSimPointArray.clear();
}

AtHit::AtHit(Int_t hitID, TVector3 vec, Double_t charge)
{
  fTrackID = -1;
  SetHit(hitID, vec, charge);

  fIsClustered = kFALSE;
  fClusterID = -1;
    fPadNum = -1;
    fQhit = -100.0;
    fHitMult = 0;
    fTimeStamp = 0;
    fTimeStampCorr = 0.0;
    fTimeStampCorrInter = 0.0;
    fBaseCorr = 0.0;
    fSlopeCnt=0;
    kIsAux = kFALSE;
    fMCSimPointArray.clear();
}

AtHit::AtHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge)
{
  fTrackID = -1;
  SetHit(hitID, x, y, z, charge);

  fIsClustered = kFALSE;
  fClusterID = -1;
    fPadNum = -1;
    fQhit = -100.0;
    fHitMult = 0;
    fTimeStamp = 0;
    fTimeStampCorr = 0.0;
    fTimeStampCorrInter = 0.0;
    fBaseCorr = 0.0;
    fSlopeCnt=0;
    kIsAux = kFALSE;
    fMCSimPointArray.clear();
}

AtHit::AtHit(Int_t PadNum,Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge)
{
    fTrackID = -1;
    SetHit(PadNum,hitID, x, y, z, charge);

    fIsClustered = kFALSE;
    fClusterID = -1;
    fQhit = -100.0;
    fHitMult = 0;
    fTimeStamp = 0;
    fTimeStampCorr = 0.0;
    fTimeStampCorrInter = 0.0;
    fBaseCorr = 0.0;
    fSlopeCnt=0;
    kIsAux = kFALSE;
    fMCSimPointArray.clear();

}

AtHit::AtHit(AtHit *hit)
{
  fTrackID = hit -> GetTrackID();
  SetHit(hit -> GetHitID(), hit -> GetPosition(), hit -> GetCharge());
  SetTimeStamp(hit->GetTimeStamp());
  SetTimeStampCorr(hit->GetTimeStampCorr());
  SetTimeStampCorrInter(hit->GetTimeStampCorrInter());
  SetSlopeCnt(hit->GetSlopeCnt());
  fIsClustered = hit -> IsClustered();
  fClusterID = hit -> GetClusterID();
  fQhit = -100.0;
  fHitMult = 0;
  fBaseCorr = 0.0;
}

AtHit::~AtHit()
{}

void AtHit::SetTrackID(Int_t trackID)                                                               { fTrackID = trackID; }
void AtHit::SetHitID(Int_t hitID)                                                                   { fHitID = hitID; }
void AtHit::SetHit(Int_t hitID, TVector3 vec, Double_t charge)                                      { fHitID = hitID; fPosition = vec; fCharge = charge; }
void AtHit::SetHit(Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge)                { fHitID = hitID; fPosition = TVector3(x, y, z); fCharge = charge; }
void AtHit::SetHit(Int_t PadNum,Int_t hitID, Double_t x, Double_t y, Double_t z, Double_t charge)   { fPadNum= PadNum ; fHitID = hitID; fPosition = TVector3(x, y, z); fCharge = charge; }

void AtHit::SetPosition(TVector3 vec)                                   { fPosition = vec; }
void AtHit::SetPosition(Double_t x, Double_t y, Double_t z)             { fPosition = TVector3(x, y, z); }
void AtHit::SetPositionCorr(Double_t x, Double_t y, Double_t z)         { fPositionCorr = TVector3(x, y, z); }
void AtHit::SetPosSigma(TVector3 vec)                                   { fPositionSigma = vec; }
void AtHit::SetPosSigma(Double_t dx, Double_t dy, Double_t dz)          { fPositionSigma = TVector3(dx, dy, dz); }
void AtHit::SetCharge(Double_t charge)                                  { fCharge = charge; }

void AtHit::SetQHit(Double_t Qhit)                                      { fQhit = Qhit;}
void AtHit::SetHitMult(Int_t HitMult)				                           	{ fHitMult = HitMult;}
void AtHit::SetTimeStamp(Int_t Time)				                           	{ fTimeStamp = Time;}
void AtHit::SetTimeStampCorr(Double_t TimeCorr)				                  { fTimeStampCorr = TimeCorr;}
void AtHit::SetTimeStampCorrInter(Double_t TimeCorrInter)				        { fTimeStampCorrInter = TimeCorrInter;}
void AtHit::SetBaseCorr(Double_t BaseCorr)                              { fBaseCorr = BaseCorr;}
void AtHit::SetSlopeCnt(Int_t cnt)                                      { fSlopeCnt = cnt;}

void AtHit::SetIsClustered(Bool_t value)                                { fIsClustered = value; }
void AtHit::SetClusterID(Int_t clusterID)                               { fClusterID = clusterID; fIsClustered = kTRUE; }

void AtHit::SetIsAux(bool value)                                        { kIsAux = value;}

Int_t AtHit::GetTrackID() const                                         { return fTrackID; }
Int_t AtHit::GetHitID() const                                           { return fHitID; }
Int_t AtHit::GetHitPadNum() const                                       { return fPadNum; }
TVector3 AtHit::GetPosition() const                                     { return fPosition; }
TVector3 AtHit::GetPositionCorr() const                                 { return fPositionCorr; }
TVector3 AtHit::GetPosSigma() const                                     { return fPositionSigma; }
Double_t AtHit::GetCharge() const                                       { return fCharge; }
Double_t AtHit::GetQHit() const                                         { return fQhit;}
Int_t AtHit::GetHitMult() const                                         { return fHitMult;}
Int_t AtHit::GetTimeStamp() const                                       { return fTimeStamp;}
Double_t AtHit::GetTimeStampCorr() const                                { return fTimeStampCorr;}
Double_t AtHit::GetTimeStampCorrInter() const                           { return fTimeStampCorrInter;}
Bool_t AtHit::IsClustered() const                                       { return fIsClustered; }
Int_t AtHit::GetClusterID() const                                       { return (fIsClustered ? fClusterID : -1); }
Double_t AtHit::GetBaseCorr() const                                     { return fBaseCorr; }
Int_t AtHit::GetSlopeCnt() const                                        { return fSlopeCnt; }
std::vector<AtHit::MCSimPoint>& AtHit::GetMCSimPointArray()		{ return fMCSimPointArray; }

bool AtHit::IsAux() const                                               { return kIsAux;}

void AtHit::SetMCSimPoint(AtHit::MCSimPoint point)
{
  fMCSimPointArray.push_back(point);
}



