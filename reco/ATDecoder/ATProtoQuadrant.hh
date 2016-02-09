#ifndef ATPROTOQUADRANT_H
#define ATPROTOQUADRANT_H

#include "TROOT.h"
#include "TObject.h"
#include "ATHit.hh"
#include "TH1D.h"
#include "TH1.h"

#include <vector>

class ATProtoQuadrant : public TObject {
  public:
	ATProtoQuadrant();
        ATProtoQuadrant(Int_t QuadrantID);
        ATProtoQuadrant(std::vector<ATHit> *HitArray,Int_t QuadrantID);
        ATProtoQuadrant(std::vector<ATHit> *HitArray,Int_t QuadrantID,Double_t PhiQ);
        ~ATProtoQuadrant();

        void SetEventID(Int_t evtid);
        void AddHit(ATHit *hit);
        void SetHitArray(std::vector<ATHit> *hitArray);
        void SetQuadrantID(Int_t QuadrantID);
        void SetPhiQ(Double_t PhiQ);
        void SetPhiDistribution(TH1D* PhiD);
        void AddPhiVal(Double_t phival);

	Int_t GetQuadrantID();
        Double_t GetPhiQ();
        Int_t GetEventID();
        Int_t GetNumHits();
        ATHit *GetHit(Int_t hitNo);
        std::vector<ATHit> *GetHitArray();
        TH1D* GetPhiDistribution();
        std::vector<Double_t> *GetPhiArray();
        Int_t GetNumPhiVal();
        
        
  protected:
        std::vector<ATHit> fHitArrayQ; // Collection of hits in that quadrant
        Double_t fPhiQ; //Phi angle on the quadrant
        Int_t fQuadrantID; //Quadrant ID : 1 (0-90) - 2 (90 - 180) -  3 (180-270) - 4 (270-360) - 0 (Central Pad) [Defined in the Phi Task]
        //TODO: A ATTrack object must be collected here...Future stuff
        Int_t fEventID;
        TH1D fPhiDistr;
	std::vector<Double_t> fPhiDistrArray;
      

  ClassDef(ATProtoQuadrant, 1);

};

#endif

