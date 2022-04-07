#include "AtPatternEvent.h"

#include <Rtypes.h>

ClassImp(AtPatternEvent);

AtPatternEvent::AtPatternEvent() : TNamed("AtPatternEvent", "Pattern Recognition Event") {}

AtPatternEvent::~AtPatternEvent() = default;

void AtPatternEvent::SetTrackCand(std::vector<AtTrack> tracks)
{
   fTrackCand = tracks;
}
std::vector<AtTrack> &AtPatternEvent::GetTrackCand()
{
   return fTrackCand;
}
