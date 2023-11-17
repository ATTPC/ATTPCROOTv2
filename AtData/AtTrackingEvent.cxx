#include "AtTrackingEvent.h"

#include <Rtypes.h>
#include <TVector3.h>

#include <algorithm>

ClassImp(AtTrackingEvent);

AtTrackingEvent::AtTrackingEvent() : AtBaseEvent("Tracking Event") {}

void AtTrackingEvent::SetTrackArray(std::vector<AtTrack> *trackArray)
{
   fTrackArray = *trackArray;
}
void AtTrackingEvent::SetTrack(AtTrack *track)
{
   fTrackArray.push_back(*track);
}
void AtTrackingEvent::SetVertex(Double_t vertex)
{
   fVertex = vertex;
}
void AtTrackingEvent::SetGeoVertex(TVector3 vertex)
{
   fGeoVertex = vertex;
}
void AtTrackingEvent::SetVertexEnergy(Double_t vertexEner)
{
   fVertexEnergy = vertexEner;
}

/*std::vector<AtTrack> AtTrackingEvent::GetTrackArray()
{
   return fTrackArray;
}*/
Double_t AtTrackingEvent::GetVertex()
{
   return fVertex;
}
Double_t AtTrackingEvent::GetVertexEnergy()
{
   return fVertexEnergy;
}
TVector3 AtTrackingEvent::GetGeoVertex()
{
   return fGeoVertex;
}
