#include "ATTrackingEventAna.hh"

ClassImp(ATTrackingEventAna);

ATTrackingEventAna::ATTrackingEventAna()
:TNamed("ATTrackingEventAna", "Tracking Event Analysis")
{


}


ATTrackingEventAna::~ATTrackingEventAna()
{

}

void ATTrackingEventAna::SetTrackArray(std::vector<ATTrack> *trackArray)        { fTrackArray = *trackArray; }
void ATTrackingEventAna::SetTrack(ATTrack *track)                               { fTrackArray.push_back(*track);}
