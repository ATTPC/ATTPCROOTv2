#include "AtPatternEvent.h"

ClassImp(AtPatternEvent);

AtPatternEvent::AtPatternEvent()
:TNamed("AtPatternEvent", "Pattern Recognition Event")
{


}


AtPatternEvent::~AtPatternEvent()
{

}

void AtPatternEvent::SetTrackCand(std::vector<AtTrack> tracks) {fTrackCand = tracks;}
std::vector<AtTrack>& AtPatternEvent::GetTrackCand() {return fTrackCand;}
