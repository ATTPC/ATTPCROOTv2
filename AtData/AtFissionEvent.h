#ifndef ATFISIONEVENT_H
#define ATFISIONEVENT_H

#include "AtBaseEvent.h" // for AtBaseEvent
#include "AtPatternEvent.h"

#include <Math/Point3D.h>     // for PositionVector3D
#include <Math/Point3Dfwd.h>  // for XYZPoint
#include <Math/Vector3D.h>    // for DisplacementVector3D
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>           // for THashConsistencyHolder, ClassDefOverride

#include <array>   // for swap, array
#include <utility> // for move
#include <vector>
class AtHit;
class AtTrack;
class TBuffer;
class TClass;
class TMemberInspector;
namespace AtPatterns {
class AtPatternY;
}

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
   Double_t fLambda{0};                //<Magnitude of the space charge correction used in this event

public:
   AtFissionEvent();
   AtFissionEvent(const AtFissionEvent &);
   AtFissionEvent(const AtPatternEvent &);
   AtFissionEvent(const AtBaseEvent &event) : AtPatternEvent(event){};
   AtFissionEvent &operator=(AtFissionEvent);
   virtual ~AtFissionEvent() = default;

   friend void swap(AtFissionEvent &first, AtFissionEvent &second)
   {
      using std::swap;
      swap(dynamic_cast<AtPatternEvent &>(first), dynamic_cast<AtPatternEvent &>(second));
      swap(first.fBeamHits, second.fBeamHits);
      swap(first.fFragHits, second.fFragHits);
   }

   const AtPatterns::AtPatternY *GetYPattern() const;
   const AtTrack &GetYTrack() const;
   double GetFoldingAngle();
   XYZPoint GetVertex() const;
   double GetLambda() const { return fLambda; }

   std::vector<AtHit *> GetBeamHits() const;
   std::vector<AtHit *> GetFragHits() const;
   std::vector<AtHit *> GetFragHits(int fragID) const;

   std::vector<AtHit *> GetBeamHitsCorr();
   std::vector<AtHit *> GetFragHitsCorr();
   std::vector<AtHit *> GetFragHitsCorr(int fragID);

   void SetBeamHits(HitVector vec) { fBeamHits = std::move(vec); }
   void SetFragHits(int fragID, HitVector vec);
   void SetLambda(double l) { fLambda = l; }
   ClassDefOverride(AtFissionEvent, 1);
};

#endif // ATFISIONEVENT_H
