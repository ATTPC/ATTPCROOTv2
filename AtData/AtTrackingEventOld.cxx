#include "AtTrackingEventOld.h"

#include <Rtypes.h>
#include <TVector3.h>

#include <algorithm>

ClassImp(AtTrackingEventOld);

AtTrackingEventOld::AtTrackingEventOld() : AtBaseEvent("Tracking Event") {}

void AtTrackingEventOld::SetTrackArray(std::vector<AtTrack> *trackArray)
{
   fTrackArray = *trackArray;
}
void AtTrackingEventOld::SetTrack(AtTrack *track)
{
   fTrackArray.push_back(*track);
}
void AtTrackingEventOld::SetVertex(Double_t vertex)
{
   fVertex = vertex;
}
void AtTrackingEventOld::SetGeoVertex(TVector3 vertex)
{
   fGeoVertex = vertex;
}
void AtTrackingEventOld::SetVertexEnergy(Double_t vertexEner)
{
   fVertexEnergy = vertexEner;
}

/*std::vector<AtTrack> AtTrackingEventOld::GetTrackArray()
{
   return fTrackArray;
}*/
Double_t AtTrackingEventOld::GetVertex()
{
   return fVertex;
}
Double_t AtTrackingEventOld::GetVertexEnergy()
{
   return fVertexEnergy;
}
TVector3 AtTrackingEventOld::GetGeoVertex()
{
   return fGeoVertex;
}
