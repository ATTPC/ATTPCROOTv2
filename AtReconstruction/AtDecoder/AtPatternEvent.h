#ifndef AtPAtTERNEVENT_H
#define AtPAtTERNEVENT_H

#include "TROOT.h"
#include "TObject.h"
#include "AtTrack.h"
#include "TVector3.h"

#include <vector>
#include <map>

class AtPatternEvent : public TNamed {

public:
   AtPatternEvent();
   ~AtPatternEvent();

   void SetTrackCand(std::vector<AtTrack> tracks);

   std::vector<AtTrack> &GetTrackCand();

private:
   std::vector<AtTrack> fTrackCand; // Candidate tracks

   ClassDef(AtPatternEvent, 1);
};

#endif
