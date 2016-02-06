#ifndef ATVertexPropagator_H
#define ATVertexPropagator_H

#include "TObject.h"

#include <iostream>
#include <map>

class ATVertexPropagator;


class ATVertexPropagator : public TObject
{

  public:


    ATVertexPropagator();
    virtual ~ATVertexPropagator();

    Bool_t Test();

    ClassDef(ATVertexPropagator,1)

   Int_t fGlobalEvtCnt;
   Int_t fBeamEvtCnt;
   Int_t fDecayEvtCnt;

   void SetVertex(Double_t vx,Double_t vy,Double_t vz,Double_t invx,Double_t invy,Double_t invz,Double_t px,Double_t py, Double_t pz, Double_t E);
   void SetBeamMass(Double_t m);
   void SetRecoilE(Double_t val);
   void SetRecoilA(Double_t val);
   void SetScatterE(Double_t val);
   void SetScatterA(Double_t val);
   void SetRndELoss(Double_t eloss);
   void SetBeamNomE(Double_t ener);
   void ResetVertex();

   Int_t GetGlobalEvtCnt();
   Int_t GetBeamEvtCnt();
   Int_t GetDecayEvtCnt();
   Double_t GetBeamMass();
   Double_t GetVx();
   Double_t GetVy();
   Double_t GetVz();
   Double_t GetInVx();
   Double_t GetInVy();
   Double_t GetInVz();
   Double_t GetPx();
   Double_t GetPy();
   Double_t GetPz();
   Double_t GetEnergy();
   Double_t GetRndELoss();
   Double_t GetBeamNomE();
   Double_t GetRecoilE();
   Double_t GetRecoilA();
   Double_t GetScatterE();
   Double_t GetScatterA();



   void IncGlobalEvtCnt();
   void IncBeamEvtCnt();
   void IncDecayEvtCnt();

   void SetValidKine(Bool_t val);
   Bool_t GetValidKine();



   Double_t fVx;
   Double_t fVy;
   Double_t fVz;
   Double_t fPx;
   Double_t fPy;
   Double_t fPz;
   Double_t fE;
   Double_t fBeamMass;
   Double_t fRndELoss;
   Double_t fBeamNomE;
   Double_t fInVx;
   Double_t fInVy;
   Double_t fInVz;
   Double_t fRecoilE;
   Double_t fRecoilA;
   Double_t fScatterE;
   Double_t fScatterA;
   Bool_t fIsValidKine;

};

extern ATVertexPropagator *gATVP; // global


#endif
