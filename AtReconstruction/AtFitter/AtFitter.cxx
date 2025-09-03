#include "AtFitter.h"

#include "AtPatternEvent.h"
#include "AtTrackingEvent.h"

ClassImp(EventFit::AtFitter);

void EventFit::AtFitter::FitEvent(AtTrackingEvent *trackingEvent, AtPatternEvent *patternEvent,
                                  AtFitMetadata *fitMetadata, AtRawEvent *rawEvent, AtEvent *event)
{
   // Extract the candidate AtTracks.
   std::vector<AtTrack> tracks = patternEvent->GetTrackCand();

   // Save the original AtTracks to the AtTrackingEvent.
   trackingEvent->SetTrackArray(&tracks);

   // Iterate over the AtTracks and store the AtFittedTracks in the AtTrackingEvent.
   for (auto track : tracks) {
      std::unique_ptr<AtFittedTrack> fittedTrack(GetFittedTrack(&track, fitMetadata, rawEvent, event));
      trackingEvent->AddFittedTrack(std::move(fittedTrack));
   }
}
