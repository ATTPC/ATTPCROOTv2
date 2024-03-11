#ifndef ATPATTERNEVENT_H
#define ATPATTERNEVENT_H

#include "AtBaseEvent.h"
#include "AtTrack.h"

#include <Rtypes.h>

#include <algorithm>
#include <memory> // for make_unique, unique_ptr
#include <utility>
#include <vector>

class AtHit;
class TBuffer;
class TClass;
class TMemberInspector;

class AtPatternEvent : public AtBaseEvent {
protected:
   using HitPtr = std::unique_ptr<AtHit>;
   using HitVector = std::vector<HitPtr>;

   std::vector<AtTrack> fTrackCand; // Candidate tracks
   HitVector fNoise;

public:
   AtPatternEvent(const char *name = "AtPatternEvent");
   AtPatternEvent(const AtPatternEvent &copy);
   AtPatternEvent(const AtBaseEvent &copy) : AtBaseEvent(copy) { SetName("AtPatternEvent"); }
   AtPatternEvent(AtPatternEvent &&copy) = default;
   AtPatternEvent &operator=(const AtPatternEvent object);
   virtual ~AtPatternEvent() = default;

   void Clear(Option_t *opt = nullptr) override;

   friend void swap(AtPatternEvent &first, AtPatternEvent &second)
   {
      using std::swap;
      swap(dynamic_cast<AtBaseEvent &>(first), dynamic_cast<AtBaseEvent &>(second));
      swap(first.fTrackCand, second.fTrackCand);
      swap(first.fNoise, second.fNoise);
   };

   void SetTrackCand(std::vector<AtTrack> tracks) { fTrackCand = std::move(tracks); }
   void AddTrack(const AtTrack &track) { fTrackCand.push_back(track); }
   void AddTrack(AtTrack &&track) { fTrackCand.push_back(track); }

   template <typename... Ts>
   void AddNoise(Ts &&... params)
   {
      fNoise.emplace_back(std::make_unique<AtHit>(std::forward<Ts>(params)...));
   }
   const HitVector &GetNoiseHits() { return fNoise; }

   std::vector<AtTrack> &GetTrackCand() { return fTrackCand; }
   const std::vector<AtTrack> &GetTrackCand() const { return fTrackCand; }

   ClassDefOverride(AtPatternEvent, 3);
};

#endif
