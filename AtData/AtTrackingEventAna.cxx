#include "AtTrackingEventAna.h"

#include <TVector3.h>
#include <algorithm>

#include <Rtypes.h>

ClassImp(AtTrackingEventAna);

AtTrackingEventAna::AtTrackingEventAna() : TNamed("AtTrackingEventAna", "Tracking Event Analysis")
{

   fVertex = -10.0;
   fVertexEnergy = -10.0;
}

AtTrackingEventAna::~AtTrackingEventAna() {}

void AtTrackingEventAna::SetTrackArray(std::vector<AtTrack> *trackArray)
{
   fTrackArray = *trackArray;
}
void AtTrackingEventAna::SetTrack(AtTrack *track)
{
   fTrackArray.push_back(*track);
}
void AtTrackingEventAna::SetVertex(Double_t vertex)
{
   fVertex = vertex;
}
void AtTrackingEventAna::SetGeoVertex(TVector3 vertex)
{
   fGeoVertex = vertex;
}
void AtTrackingEventAna::SetVertexEnergy(Double_t vertexEner)
{
   fVertexEnergy = vertexEner;
}

std::vector<AtTrack> AtTrackingEventAna::GetTrackArray()
{
   return fTrackArray;
}
Double_t AtTrackingEventAna::GetVertex()
{
   return fVertex;
}
Double_t AtTrackingEventAna::GetVertexEnergy()
{
   return fVertexEnergy;
}
TVector3 AtTrackingEventAna::GetGeoVertex()
{
   return fGeoVertex;
}
