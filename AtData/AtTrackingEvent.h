#ifndef AtTRACKINGEVENT_H
#define AtTRACKINGEVENT_H

#include "AtBaseEvent.h"
#include "AtFittedTrack.h"
#include "AtTrack.h"

#include <Rtypes.h>
#include <TNamed.h>
#include <TVector3.h>

#include <vector>

class TBuffer;
class TClass;
class TMemberInspector;

class AtTrackingEvent : public AtBaseEvent {

   using FTrackPtr = std::unique_ptr<AtFittedTrack>;
   using FTrackVector = std::vector<FTrackPtr>;

public:
   AtTrackingEvent();
   virtual ~AtTrackingEvent() = default;

   void SetTrackArray(std::vector<AtTrack> *trackArray);
   void SetTrack(AtTrack *track);
   void SetVertex(Double_t vertex);
   void SetGeoVertex(TVector3 vertex);
   void SetVertexEnergy(Double_t vertexEner);

   AtFittedTrack &AddFittedTrack(std::unique_ptr<AtFittedTrack> ptr)
   {
      fFittedTrackArray.push_back(std::move(ptr));
      if (fFittedTrackArray.back()->GetTrackID() == -1)
         fFittedTrackArray.back()->SetTrackID(fFittedTrackArray.size() - 1);
      LOG(debug) << "Adding Track with ID " << fFittedTrackArray.back()->GetTrackID() << " to event " << fEventID;

      return *(fFittedTrackArray.back());
   }

   Double_t GetVertex();
   Double_t GetVertexEnergy();
   TVector3 GetGeoVertex();
   std::vector<AtTrack> GetTrackArray();
   const FTrackVector &GetFittedTracks() const { return fFittedTrackArray; }

private:
   std::vector<AtTrack> fTrackArray;
   FTrackVector fFittedTrackArray;
   Double_t fVertex{-10.0};
   Double_t fVertexEnergy{-10.0};
   TVector3 fGeoVertex;

   ClassDef(AtTrackingEvent, 1);
};

#endif
