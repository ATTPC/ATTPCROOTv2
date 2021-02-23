#ifndef AtPHIRECOSIMPLE_H
#define AtPHIRECOSIMPLE_H

#include "AtPhiReco.h"
#include "AtProtoEvent.h"
#include "AtProtoQuadrant.h"

class AtPhiRecoSimple : public AtPhiReco
{
  public:
    AtPhiRecoSimple();
    ~AtPhiRecoSimple();
   
    void PhiAnalyze(AtEvent *event,AtProtoEvent *protoevent);
    //void PhiAnalyze(AtEvent *event, AtHoughSpace *HSpace);

   private:
   void PhiCalc(AtProtoQuadrant *quadrant,AtEvent *event);
   void PhiCalcMulti(std::vector<AtHit> *multihit_Array,AtProtoQuadrant *quadrant);
   TH1D* PhiDist; 

  ClassDef(AtPhiRecoSimple, 1)
};

#endif
