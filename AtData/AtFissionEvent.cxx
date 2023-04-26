#include "AtFissionEvent.h"

#include "AtContainerManip.h"
#include "AtHit.h"     // for AtHit
#include "AtPattern.h" // for AtPattern
#include "AtPatternY.h"
#include "AtTrack.h" // for AtTrack, AtTrack::HitVector

#include <Math/VectorUtil.h>

#include <algorithm> // for find
#include <cassert>
#include <memory>    // for unique_ptr
#include <stdexcept> // for invalid_argument

ClassImp(AtFissionEvent);

AtFissionEvent::AtFissionEvent() : AtPatternEvent("AtFissionEvent") {}

AtFissionEvent::AtFissionEvent(const AtFissionEvent &obj) : AtPatternEvent(obj)
{
   for (auto &hit : obj.fBeamHits)
      fBeamHits.push_back(hit->Clone());
   for (int i = 0; i < obj.fFragHits.size(); ++i)
      for (auto &hit : obj.fFragHits[i])
         fFragHits[i].push_back(hit->Clone());
}
AtFissionEvent::AtFissionEvent(const AtPatternEvent &obj) : AtPatternEvent(obj) {}

AtFissionEvent &AtFissionEvent::operator=(AtFissionEvent other)
{
   swap(*this, other);
   return *this;
}

const AtPatterns::AtPatternY *AtFissionEvent::GetYPattern() const
{
   const AtPatterns::AtPatternY *patt = nullptr;
   try {
      patt = dynamic_cast<const AtPatterns::AtPatternY *>(GetYTrack().GetPattern());
   } catch (...) {
   }
   return patt;
}

const AtTrack &AtFissionEvent::GetYTrack() const
{
   for (auto &track : fTrackCand) {
      auto pat = dynamic_cast<const AtPatterns::AtPatternY *>(track.GetPattern());
      if (pat != nullptr)
         return track;
   }
   throw std::invalid_argument("This event does not contain a Y pattern describing a fission event.");
}

double AtFissionEvent::GetFoldingAngle()
{
   auto yPatt = GetYPattern();
   if (yPatt)
      return ROOT::Math::VectorUtil::Angle(yPatt->GetFragmentDirection(0), yPatt->GetFragmentDirection(1));
   return -1;
}

AtFissionEvent::XYZPoint AtFissionEvent::GetVertex() const
{
   auto yPatt = GetYPattern();
   if (yPatt)
      return yPatt->GetVertex();
   else
      return {-1000, -1000, -1000};
}

/**
 * Get the hits associated with the beam that have not been space-charge corrected.
 * (The hits stored in this sub-class)
 */
std::vector<AtHit *> AtFissionEvent::GetBeamHits() const
{
   return ContainerManip::GetPointerVector(fBeamHits);
}

/**
 * Get the hits associated with both fission fragments that have not been space-charge corrected.
 * (The hits stored in this sub-class)
 */
std::vector<AtHit *> AtFissionEvent::GetFragHits() const
{
   auto ret = ContainerManip::GetPointerVector(fFragHits[0]);
   for (auto &hit : fFragHits[1])
      ret.push_back(hit.get());
   return ret;
}

/**
 * Get the hits associated with both fission fragments that have not been space-charge corrected.
 * (The hits stored in this sub-class)
 */
std::vector<AtHit *> AtFissionEvent::GetFragHits(int fragID) const
{
   assert(fragID >= 0 && fragID < 2);
   return ContainerManip::GetPointerVector(fFragHits[fragID]);
}

/**
 * Get the hits associated with the beam that belong to the track in this event (typically the ones
 * that are space charge corrected).
 */
std::vector<AtHit *> AtFissionEvent::GetBeamHitsCorr()
{
   auto &track = GetYTrack();
   auto pat = GetYPattern();

   std::vector<AtHit *> ret;
   for (auto &hit : track.GetHitArray()) {
      if (pat->GetPointAssignment(hit->GetPosition()) == 2)
         ret.push_back(hit.get());
   }
   return ret;
}

/**
 * Get the hits associated with the fragment that belong to the track in this event (typically the ones
 * that are space charge corrected).
 */
std::vector<AtHit *> AtFissionEvent::GetFragHitsCorr(int i)
{
   assert(i >= 0 && i < 2);
   auto &track = GetYTrack();
   auto pat = GetYPattern();

   std::vector<AtHit *> ret;
   for (auto &hit : track.GetHitArray()) {
      if (pat->GetPointAssignment(hit->GetPosition()) == i)
         ret.push_back(hit.get());
   }
   return ret;
}

/**
 * Get the hits associated with the fragment that belong to the track in this event (typically the ones
 * that are space charge corrected).
 */
std::vector<AtHit *> AtFissionEvent::GetFragHitsCorr()
{
   auto ret = GetFragHitsCorr(0);

   for (auto &hit : GetFragHitsCorr(1)) {
      ret.push_back(hit);
   }
   return ret;
}

void AtFissionEvent::SetFragHits(int fragID, HitVector vec)
{

   assert(fragID >= 0 && fragID < 2);
   fFragHits[fragID] = std::move(vec);
}
