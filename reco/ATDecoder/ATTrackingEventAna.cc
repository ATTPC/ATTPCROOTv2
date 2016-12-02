#include "ATTrackingEventAna.hh"

ClassImp(ATTrackingEventAna);

ATTrackingEventAna::ATTrackingEventAna()
:TNamed("ATTrackingEventAna", "Tracking Event Analysis")
{

    fVertex = -10.0;
    fVertexEnergy = -10.0;

}


ATTrackingEventAna::~ATTrackingEventAna()
{

}

void ATTrackingEventAna::SetTrackArray(std::vector<ATTrack> *trackArray)        { fTrackArray = *trackArray; }
void ATTrackingEventAna::SetTrack(ATTrack *track)                               { fTrackArray.push_back(*track);}
void ATTrackingEventAna::SetVertex(Double_t vertex)                             { fVertex = vertex;}
void ATTrackingEventAna::SetVertexEnergy(Double_t vertexEner)                   { fVertexEnergy = vertexEner;}

std::vector<ATTrack> ATTrackingEventAna::GetTrackArray()                        { return fTrackArray;}
Double_t ATTrackingEventAna::GetVertex()                                        { return fVertex;}
Double_t ATTrackingEventAna::GetVertexEnergy()                                  { return fVertexEnergy;}
