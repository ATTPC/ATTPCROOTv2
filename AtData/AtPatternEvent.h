#ifndef ATPATTERNEVENT_H
#define ATPATTERNEVENT_H

#include "AtTrack.h"

#include <Rtypes.h>
#include <TNamed.h>

#include <algorithm>
#include <vector>
class TBuffer;
class TClass;
class TMemberInspector;

class AtPatternEvent : public TNamed {
private:
   std::vector<AtTrack> fTrackCand; // Candidate tracks
   std::vector<AtHit> fNoise;

public:
   AtPatternEvent();
   ~AtPatternEvent();

   void SetTrackCand(std::vector<AtTrack> tracks);
   void AddTrack(const AtTrack &track) { fTrackCand.push_back(track); }
   void AddTrack(AtTrack &&track) { fTrackCand.push_back(track); }

   void AddNoise(AtHit hit) { fNoise.push_back(std::move(hit)); }
   const std::vector<AtHit> &GetNoiseHits() { return fNoise; }
   std::vector<AtTrack> &GetTrackCand();

   ClassDef(AtPatternEvent, 2);
};

#endif
