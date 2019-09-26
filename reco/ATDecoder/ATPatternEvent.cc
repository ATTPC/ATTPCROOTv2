#include "ATPatternEvent.hh"

ClassImp(ATPatternEvent);

ATPatternEvent::ATPatternEvent()
:TNamed("ATPatternEvent", "Pattern Recognition Event")
{


}


ATPatternEvent::~ATPatternEvent()
{

}

void ATPatternEvent::SetTrackCand(std::vector<ATTrack> tracks) {fTrackCand = tracks;}
std::vector<ATTrack>& ATPatternEvent::GetTrackCand() {return fTrackCand;}
