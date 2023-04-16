#ifndef ATFISIONEVENT_H
#define ATFISIONEVENT_H

#include "AtPatternEvent.h"
#include "AtPatternY.h"

#include <Math/Point3D.h>     // for PositionVector3D
#include <Math/Point3Dfwd.h>  // for XYZPoint
#include <Math/Vector3D.h>    // for DisplacementVector3D
#include <Math/Vector3Dfwd.h> // for XYZVector
#include <Rtypes.h>           // for THashConsistencyHolder, ClassDefOverride

#include <array>   // for swap, array
#include <memory>  // for shared_ptr
#include <utility> // for move
#include <vector>
class AtHit;
class AtTrack;
class TBuffer;
class TClass;
class TMemberInspector;
struct AtPadReference;

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
   Double_t fLambda{0};                //<Magnitude of the space charge correction used in this event

public:
   AtFissionEvent();
   AtFissionEvent(const AtFissionEvent &);
   AtFissionEvent(const AtPatternEvent &);
   AtFissionEvent &operator=(AtFissionEvent);
   virtual ~AtFissionEvent() = default;

   friend void swap(AtFissionEvent &first, AtFissionEvent &second)
   {
      using std::swap;
      swap(dynamic_cast<AtPatternEvent &>(first), dynamic_cast<AtPatternEvent &>(second));
      swap(first.fBeamHits, second.fBeamHits);
      swap(first.fFragHits, second.fFragHits);
   }

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
   void SetLambda(double l) { fLambda = l; }
   ClassDefOverride(AtFissionEvent, 1);
};

#endif // ATFISIONEVENT_H
