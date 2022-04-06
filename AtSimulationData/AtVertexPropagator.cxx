#include "AtVertexPropagator.h"

#include <math.h>
#include <iostream>
#include <utility>

#include "TVector3.h"
#include "TRandom.h"
#include "Rtypes.h"

AtVertexPropagator *gAtVP = nullptr;
// AtVertexPropagator *gAtVP = new AtVertexPropagator();

AtVertexPropagator::AtVertexPropagator()
   : fGlobalEvtCnt(0), fBeamEvtCnt(0), fDecayEvtCnt(0), fVx(0.), fVy(0.), fVz(0.), fPx(0.), fPy(0.), fPz(0.), fE(0.),
     fBeamMass(0), fRndELoss(0), fBeamNomE(0), fInVx(0), fInVy(0), fInVz(0), fRecoilE(0), fRecoilA(0), fScatterE(0),
     fScatterA(0), fBURes1E(0), fBURes1A(0), fBURes2E(0), fBURes2A(0), fIsValidKine(0), fAiso(0), fZiso(0),
     fExEjectile(0)
{

   fScatP(0) = 0.0;
   fScatP(1) = 0.0;
   fScatP(2) = 0.0;

   if (gAtVP)
      delete gAtVP;
   gAtVP = this;
}

AtVertexPropagator::~AtVertexPropagator()
{

   delete gAtVP;
   std::cout << " AtVertexPropagator: Global pointer AtVertexPropagator succesfully deleted " << std::endl;
}

void AtVertexPropagator::SetVertex(Double_t vx, Double_t vy, Double_t vz, Double_t invx, Double_t invy, Double_t invz,
                                   Double_t px, Double_t py, Double_t pz, Double_t E)
{
   fVx = vx;
   fVy = vy;
   fVz = vz;
   fInVx = invx;
   fInVy = invy;
   fInVz = invz;
   fPx = px;
   fPy = py;
   fPz = pz;
   fE = E;
}

void AtVertexPropagator::SetRndELoss(Double_t eloss)
{
   fRndELoss = eloss;
}

void AtVertexPropagator::SetBeamNomE(Double_t nome)
{
   fBeamNomE = nome;
}

void AtVertexPropagator::ResetVertex()
{

   fVx = 0.0;
   fVy = 0.0;
   fVz = 0.0;
   fInVx = 0.0;
   fInVy = 0.0;
   fInVz = 0.0;
   fPx = 0.0;
   fPy = 0.0;
   fPz = 0.0;
   fE = 0.0;
   fScatP(0) = 0.0;
   fScatP(1) = 0.0;
   fScatP(2) = 0.0;
   fIsValidKine = kTRUE;

   fTrackAngle.clear();
   fTrackEn.clear();
}

void AtVertexPropagator::SetBeamMass(Double_t m)
{
   fBeamMass = m;
}

void AtVertexPropagator::SetTrackEnergy(int trackID, double energy)
{
   fTrackEn[trackID] = energy;
}
void AtVertexPropagator::SetTrackAngle(int trackID, double angle)
{
   fTrackAngle[trackID] = angle;
}

Double_t AtVertexPropagator::GetTrackAngle(int trackID)
{
   auto it = fTrackAngle.find(trackID);
   if (it == fTrackAngle.end())
      return 0;
   else
      return it->second;
}
Double_t AtVertexPropagator::GetTrackEnergy(int trackID)
{
   auto it = fTrackEn.find(trackID);
   if (it == fTrackEn.end())
      return 0;
   else
      return it->second;
}

/*
void AtVertexPropagator::SetRecoilE(Double_t val)
{
   fRecoilE = val;
}
void AtVertexPropagator::SetRecoilA(Double_t val)
{
   fRecoilA = val;
}
void AtVertexPropagator::SetScatterE(Double_t val)
{
   fScatterE = val;
}
void AtVertexPropagator::SetScatterA(Double_t val)
{
   fScatterA = val;
}
void AtVertexPropagator::SetBURes1E(Double_t val)
{
   fBURes1E = val;
}
void AtVertexPropagator::SetBURes1A(Double_t val)
{
   fBURes1A = val;
}
void AtVertexPropagator::SetBURes2E(Double_t val)
{
   fBURes2E = val;
}
void AtVertexPropagator::SetBURes2A(Double_t val)
{
   fBURes2A = val;
}
*/

void AtVertexPropagator::SetMassNum(Int_t mnum)
{
   fAiso = mnum;
}
void AtVertexPropagator::SetAtomicNum(Int_t anum)
{
   fZiso = anum;
}
void AtVertexPropagator::SetScatterP(TVector3 avec)
{
   fScatP = avec;
}
void AtVertexPropagator::SetScatterEx(Double_t val)
{
   fExEjectile = val;
}
void AtVertexPropagator::Setd2HeVtx(TVector3 avec)
{
   fd2HeVtx = avec;
}
void AtVertexPropagator::Setd2HeVtx(Double_t x0, Double_t y0, Double_t theta, Double_t phi)
{
   Double_t vx, vy, vz;
   vz = 100.0 * (gRandom->Uniform()); // cm
   // vz=50.;
   vx = x0 + vz * tan(phi);
   vy = y0 + sqrt(pow(vz, 2) + pow(vx - x0, 2)) * tan(theta);
   fd2HeVtx.SetXYZ(vx, vy, vz);
}

Int_t AtVertexPropagator::GetGlobalEvtCnt()
{
   return fGlobalEvtCnt;
}
Int_t AtVertexPropagator::GetBeamEvtCnt()
{
   return fBeamEvtCnt;
}
Int_t AtVertexPropagator::GetDecayEvtCnt()
{
   return fDecayEvtCnt;
}
Double_t AtVertexPropagator::GetVx()
{
   return fVx;
}
Double_t AtVertexPropagator::GetVy()
{
   return fVy;
}
Double_t AtVertexPropagator::GetVz()
{
   return fVz;
}
Double_t AtVertexPropagator::GetInVx()
{
   return fInVx;
}
Double_t AtVertexPropagator::GetInVy()
{
   return fInVy;
}
Double_t AtVertexPropagator::GetInVz()
{
   return fInVz;
}
Double_t AtVertexPropagator::GetPx()
{
   return fPx;
}
Double_t AtVertexPropagator::GetPy()
{
   return fPy;
}
Double_t AtVertexPropagator::GetPz()
{
   return fPz;
}
Double_t AtVertexPropagator::GetEnergy()
{
   return fE;
}
Double_t AtVertexPropagator::GetBeamMass()
{
   return fBeamMass;
}
Double_t AtVertexPropagator::GetRndELoss()
{
   return fRndELoss;
}
Double_t AtVertexPropagator::GetBeamNomE()
{
   return fBeamNomE;
}

/*
Double_t AtVertexPropagator::GetRecoilE()
{
   return fRecoilE;
}
Double_t AtVertexPropagator::GetRecoilA()
{
   return fRecoilA;
}
Double_t AtVertexPropagator::GetScatterE()
{
   return fScatterE;
}
Double_t AtVertexPropagator::GetScatterA()
{
   return fScatterA;
}
Double_t AtVertexPropagator::GetBURes1E()
{
   return fBURes1E;
}
Double_t AtVertexPropagator::GetBURes1A()
{
   return fBURes1A;
}
Double_t AtVertexPropagator::GetBURes2E()
{
   return fBURes2E;
}
Double_t AtVertexPropagator::GetBURes2A()
{
   return fBURes2A;
}
*/

Bool_t AtVertexPropagator::GetValidKine()
{
   return fIsValidKine;
}
Int_t AtVertexPropagator::GetMassNum()
{
   return fAiso;
}
Int_t AtVertexPropagator::GetAtomicNum()
{
   return fZiso;
}
TVector3 AtVertexPropagator::GetScatterP()
{
   return fScatP;
}
Double_t AtVertexPropagator::GetScatterEx()
{
   return fExEjectile;
}
TVector3 AtVertexPropagator::Getd2HeVtx()
{
   return fd2HeVtx;
}

void AtVertexPropagator::IncGlobalEvtCnt()
{
   fGlobalEvtCnt++;
}
void AtVertexPropagator::IncBeamEvtCnt()
{
   fBeamEvtCnt++;
}
void AtVertexPropagator::IncDecayEvtCnt()
{
   fDecayEvtCnt++;
}
void AtVertexPropagator::SetValidKine(Bool_t val)
{
   fIsValidKine = val;
}

ClassImp(AtVertexPropagator);
