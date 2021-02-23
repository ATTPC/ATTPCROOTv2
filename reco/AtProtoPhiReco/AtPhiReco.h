#ifndef AtPHIRECO_H
#define AtPHIRECO_H

#include "AtRawEvent.h"
#include "AtPad.h"
#include "AtEvent.h"
#include "AtProtoEvent.h"
#include "AtHit.h"
#include "AtHoughSpace.h"
#include "AtProtoQuadrant.h"

// FairRoot classes
#include "FairRootManager.h"
#include "FairLogger.h"

// STL
#include <vector>
#include <algorithm>

// ROOT classes
#include "TClonesArray.h"


class AtPhiReco
{
  
   public:
    AtPhiReco();
    virtual ~AtPhiReco();
    virtual void PhiAnalyze(AtEvent *event,AtProtoEvent *protoevent) = 0;
    //virtual void PhiAnalyze(AtEvent *event, AtHoughSpace *HSpace) = 0; //TODO: Overload pure virtual functions???? 

    //TODO Ideas : Maximum separation in time buckets between two pulses - Sort the Hit Array vector 
   
   protected:
     std::vector<AtProtoQuadrant> fQuadArray; //Limited to 4 
     //Double_t PhiCalc(AtProtoQuadrant *quadrant); // Calculates Phi Angle of two neighboring strips
 

  ClassDef(AtPhiReco,1)
};

#endif
