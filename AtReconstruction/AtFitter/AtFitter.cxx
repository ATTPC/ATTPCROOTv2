#include "AtFitter.h"

#include "AtPatternEvent.h"
#include "AtTrackingEvent.h"

void EventFit::AtFitter::FitEvent(AtTrackingEvent *trackingEvent, AtPatternEvent *patternEvent,
                                  AtFitMetadata *fitMetadata, AtRawEvent *rawEvent, AtEvent *event)
{
   // Check for nullptr.
   if (trackingEvent == nullptr) {
      LOG(error) << " Tracking event is nullptr! The fitter can not fit this event. Maybe the tracking event is not "
                    "being constructed properly in the fitter task.";
      return;
   }

   if (patternEvent == nullptr) {
      LOG(error) << " Pattern event is nullptr! The fitter can not fit this event.";
      return;
   }

   // Extract the candidate AtTracks. If there are not any tracks, return earlier.
   std::vector<AtTrack> tracks = patternEvent->GetTrackCand();
   if (!tracks.size())
      return;

   // Save the original AtTracks to the AtTrackingEvent.
   trackingEvent->SetTrackArray(&tracks);

   // Iterate over the AtTracks and store the AtFittedTracks in the AtTrackingEvent.
   for (auto track : tracks) {
      std::unique_ptr<AtFittedTrack> fittedTrack(GetFittedTrack(&track, fitMetadata, rawEvent, event));
      trackingEvent->AddFittedTrack(std::move(fittedTrack));
   }
}
