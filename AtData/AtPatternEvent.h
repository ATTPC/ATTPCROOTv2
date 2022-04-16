#ifndef AtPAtTERNEVENT_H
#define AtPAtTERNEVENT_H

#include "AtTrack.h"

#include <Rtypes.h>
#include <TNamed.h>

#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;

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
