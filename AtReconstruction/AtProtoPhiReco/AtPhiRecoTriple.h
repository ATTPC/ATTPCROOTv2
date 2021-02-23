#ifndef AtPHIRECOTRIPLE_H
#define AtPHIRECOTRIPLE_H

#include "AtPhiReco.h"
#include "AtProtoEvent.h"
#include "AtProtoQuadrant.h"

class AtPhiRecoTriple : public AtPhiReco {
public:
   AtPhiRecoTriple();
   ~AtPhiRecoTriple();

   void PhiAnalyze(AtEvent *event, AtProtoEvent *protoevent);
   // void PhiAnalyze(AtEvent *event, AtHoughSpace *HSpace);

   friend Bool_t operator==(const AtHit &n1, const AtHit &n2);

private:
   void PhiCalc(AtProtoQuadrant *quadrant, AtEvent *event);
   void PhiCalcMulti(std::vector<AtHit> *multihit_Array, AtProtoQuadrant *quadrant);
   TH1D *PhiDist;

   ClassDef(AtPhiRecoTriple, 1)
};

#endif
