#ifndef ATPHIRECOSIMPLE_H
#define ATPHIRECOSIMPLE_H

#include "ATPhiReco.hh"
#include "ATProtoEvent.hh"
#include "ATProtoQuadrant.hh"

class ATPhiRecoSimple : public ATPhiReco
{
  public:
    ATPhiRecoSimple();
    ~ATPhiRecoSimple();
   
    void PhiAnalyze(ATEvent *event,ATProtoEvent *protoevent);
    //void PhiAnalyze(ATEvent *event, ATHoughSpace *HSpace);

   private:
   void PhiCalc(ATProtoQuadrant *quadrant,ATEvent *event);
   void PhiCalcMulti(std::vector<ATHit> *multihit_Array,ATProtoQuadrant *quadrant);
   TH1D* PhiDist; 

  ClassDef(ATPhiRecoSimple, 1)
};

#endif
