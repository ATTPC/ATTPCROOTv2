#ifndef ATFISIONEVENT_H
#define ATFISIONEVENT_H

#include "AtPadReference.h" // for AtPadReference
#include "AtPatternEvent.h"
#include "AtPatternY.h"

#include <vector>
namespace AtPatterns {
class AtPatternY;
}
class AtMap;

/**
 * Class for getting useful information from fission events.
 * Filled by the task AtFissionAnalysis.
 */
class AtFissionEvent : public AtPatternEvent {
protected:
   using XYZPoint = ROOT::Math::XYZPoint;
   using XYZVector = ROOT::Math::XYZVector;

   HitVector fBeamHits;                //< Beam hits without space charge correction
   std::array<HitVector, 2> fFragHits; //< Fragment hits without space charge correction

public:
   AtFissionEvent();
   virtual ~AtFissionEvent() = default;

   const AtPatterns::AtPatternY *GetYPattern();
   const AtTrack &GetYTrack();
   double GetFoldingAngle();
   XYZPoint GetVertex() { return GetYPattern()->GetVertex(); }

   std::vector<AtHit *> GetBeamHits();
   std::vector<AtHit *> GetFragHits();
   std::vector<AtHit *> GetFragHits(int fragID);

   std::vector<AtHit *> GetBeamHitsCorr();
   std::vector<AtHit *> GetFragHitsCorr();
   std::vector<AtHit *> GetFragHitsCorr(int fragID);

   std::vector<double>
   GetdQdTB(int fragID, const std::vector<AtPadReference> &vetoPads, std::shared_ptr<AtMap> fMap, int threshold = 15);

   void SetBeamHits(HitVector vec) { fBeamHits = std::move(vec); }
   void SetFragHits(int fragID, HitVector vec);

   ClassDefOverride(AtFissionEvent, 1);
};

#endif // ATFISIONEVENT_H
