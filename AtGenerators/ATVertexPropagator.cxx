#include "ATVertexPropagator.h"

ATVertexPropagator *gATVP = (ATVertexPropagator *)0;
//ATVertexPropagator *gATVP = new ATVertexPropagator();

// -----   Default constructor   -------------------------------------------
ATVertexPropagator::ATVertexPropagator():fGlobalEvtCnt(0),fBeamEvtCnt(0),fDecayEvtCnt(0),
fVx(0.),fVy(0.),fVz(0.),fPx(0.),fPy(0.),fPz(0.),fE(0.),fBeamMass(0),fRndELoss(0),fBeamNomE(0),fInVx(0),fInVy(0),fInVz(0),
fRecoilE(0),fRecoilA(0),fScatterE(0),fScatterA(0),fBURes1E(0),fBURes1A(0), fBURes2E(0),fBURes2A(0),fIsValidKine(0),fAiso(0),fZiso(0)
{

   if(gATVP)
    delete gATVP;
   gATVP = this;

}

// -------------------------------------------------------------------------



// -----   Destructor   ----------------------------------------------------
ATVertexPropagator::~ATVertexPropagator()
{

  delete gATVP;
  std::cout<<" ATVertexPropagator: Global pointer ATVertexPropagator succesfully deleted "<<std::endl;

}

Bool_t ATVertexPropagator::Test(){


   return kTRUE;

}


void ATVertexPropagator::SetVertex(Double_t vx,Double_t vy,Double_t vz,Double_t invx,Double_t invy,Double_t invz,Double_t px,Double_t py, Double_t pz, Double_t E)
{
    fVx=vx;
    fVy=vy;
    fVz=vz;
    fInVx=invx;
    fInVy=invy;
    fInVz=invz;
    fPx=px;
    fPy=py;
    fPz=pz;
    fE=E;
}

void ATVertexPropagator::SetRndELoss(Double_t eloss)
{
    fRndELoss=eloss;
}

void ATVertexPropagator::SetBeamNomE(Double_t nome)
{
    fBeamNomE=nome;
}

void ATVertexPropagator::ResetVertex()
{

    fVx=0.0;
    fVy=0.0;
    fVz=0.0;
    fInVx=0.0;
    fInVy=0.0;
    fInVz=0.0;
    fPx=0.0;
    fPy=0.0;
    fPz=0.0;
    fE=0.0;
    fIsValidKine=kTRUE;

}

void ATVertexPropagator::SetBeamMass(Double_t m)        { fBeamMass = m;}
void ATVertexPropagator::SetRecoilE(Double_t val)	     	{ fRecoilE = val;}
void ATVertexPropagator::SetRecoilA(Double_t val)	     	{ fRecoilA = val;}
void ATVertexPropagator::SetScatterE(Double_t val)		  { fScatterE = val;}
void ATVertexPropagator::SetScatterA(Double_t val)		  { fScatterA = val;}
void ATVertexPropagator::SetBURes1E(Double_t val)		  { fBURes1E = val;}
void ATVertexPropagator::SetBURes1A(Double_t val)		  { fBURes1A = val;}
void ATVertexPropagator::SetBURes2E(Double_t val)		  { fBURes2E = val;}
void ATVertexPropagator::SetBURes2A(Double_t val)		  { fBURes2A = val;}
void ATVertexPropagator::SetMassNum(Int_t mnum)		  { fAiso = mnum;}
void ATVertexPropagator::SetAtomicNum(Int_t anum)		{ fZiso = anum;}



Int_t ATVertexPropagator::GetGlobalEvtCnt()    			{ return fGlobalEvtCnt;}
Int_t ATVertexPropagator::GetBeamEvtCnt()    			{ return fBeamEvtCnt;}
Int_t ATVertexPropagator::GetDecayEvtCnt()    			{ return fDecayEvtCnt;}
Double_t ATVertexPropagator::GetVx()				{ return fVx;}
Double_t ATVertexPropagator::GetVy()				{ return fVy;}
Double_t ATVertexPropagator::GetVz()				{ return fVz;}
Double_t ATVertexPropagator::GetInVx()				{ return fInVx;}
Double_t ATVertexPropagator::GetInVy()				{ return fInVy;}
Double_t ATVertexPropagator::GetInVz()				{ return fInVz;}
Double_t ATVertexPropagator::GetPx()				{ return fPx;}
Double_t ATVertexPropagator::GetPy()				{ return fPy;}
Double_t ATVertexPropagator::GetPz()				{ return fPz;}
Double_t ATVertexPropagator::GetEnergy()			{ return fE;}
Double_t ATVertexPropagator::GetBeamMass()                      { return fBeamMass;}
Double_t ATVertexPropagator::GetRndELoss()                      { return fRndELoss; }
Double_t ATVertexPropagator::GetBeamNomE()                      { return fBeamNomE; }
Double_t ATVertexPropagator::GetRecoilE()			{ return fRecoilE;}
Double_t ATVertexPropagator::GetRecoilA()			{ return fRecoilA;}
Double_t ATVertexPropagator::GetScatterE()			                  { return fScatterE;}
Double_t ATVertexPropagator::GetScatterA()			                  { return fScatterA ;}
Double_t ATVertexPropagator::GetBURes1E()			                  { return fBURes1E;}
Double_t ATVertexPropagator::GetBURes1A()			                  { return fBURes1A ;}
Double_t ATVertexPropagator::GetBURes2E()			                  { return fBURes2E;}
Double_t ATVertexPropagator::GetBURes2A()			                  { return fBURes2A ;}
Bool_t ATVertexPropagator::GetValidKine()                         {  return fIsValidKine; }
Int_t ATVertexPropagator::GetMassNum()    			 { return fAiso;}
Int_t ATVertexPropagator::GetAtomicNum()    			{ return fZiso;}


void ATVertexPropagator::IncGlobalEvtCnt()                      {  fGlobalEvtCnt++;    }
void ATVertexPropagator::IncBeamEvtCnt()                        {  fBeamEvtCnt++;    }
void ATVertexPropagator::IncDecayEvtCnt()                       {  fDecayEvtCnt++;    }
void ATVertexPropagator::SetValidKine(Bool_t val)               {  fIsValidKine=val; }



ClassImp(ATVertexPropagator)
