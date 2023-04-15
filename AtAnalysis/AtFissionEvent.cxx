#include "AtFissionEvent.h"

#include "AtContainerManip.h"
#include "AtDataManip.h"
#include "AtMap.h"

#include <Math/VectorUtil.h>
#include <TF1.h>

#include <cassert>
#include <set>

ClassImp(AtFissionEvent);

AtFissionEvent::AtFissionEvent() : AtPatternEvent("AtFissionEvent") {}

const AtPatterns::AtPatternY *AtFissionEvent::GetYPattern()
{
   return dynamic_cast<const AtPatterns::AtPatternY *>(GetYTrack().GetPattern());
}

const AtTrack &AtFissionEvent::GetYTrack()
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
   return ROOT::Math::VectorUtil::Angle(GetYPattern()->GetFragmentDirection(0), GetYPattern()->GetFragmentDirection(1));
}

/**
 * Get the hits associated with the beam that have not been space-charge corrected.
 * (The hits stored in this sub-class)
 */
std::vector<AtHit *> AtFissionEvent::GetBeamHits()
{
   return ContainerManip::GetPointerVector(fBeamHits);
}

/**
 * Get the hits associated with both fission fragments that have not been space-charge corrected.
 * (The hits stored in this sub-class)
 */
std::vector<AtHit *> AtFissionEvent::GetFragHits()
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
std::vector<AtHit *> AtFissionEvent::GetFragHits(int fragID)
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

/**
 * Get the charge (Q) as a function of timebucket for a fission fragment. Will ignore any hits generated by the pads in
 * vetoPads. (along with all hits in cobo 2, asad 3)
 */
std::vector<double> AtFissionEvent::GetdQdTB(int fragID, const std::vector<AtPadReference> &vetoPads,
                                             std::shared_ptr<AtMap> fMap, int threshold)
{
   std::vector<double> ret(512, 0);

   for (auto &hit : GetFragHits(fragID)) {
      // Get the pad ref and check if it is good
      auto padNum = hit->GetPadNum();
      auto padRef = fMap->GetPadRef(padNum);
      // Throw out pad AsAd
      if (padRef.cobo == 2 && padRef.asad == 3)
         continue;
      // Throw out vetoed pads
      if (std::find(vetoPads.begin(), vetoPads.end(), padRef) != vetoPads.end())
         continue;

      auto func = AtTools::GetHitFunctionTB(*hit);
      if (func == nullptr)
         continue;

      for (int tb = 0; tb < 512; ++tb) {
         auto val = func->Eval(tb);
         if (val > threshold)
            ret[tb] += val;
      }
   }

   return ret;
}
