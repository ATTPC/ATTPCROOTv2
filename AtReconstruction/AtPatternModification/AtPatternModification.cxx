#include "AtPatternModification.h"

void AtPatternModification::ModifyPatternEvent(AtPatternEvent *patternEvent, AtRawEvent *rawEvent, AtEvent *event)
{
   // Extract the candidate AtTracks.
   std::vector<AtTrack> tracks = patternEvent->GetTrackCand();

   // Create a new vector to store the modified AtTracks.
   std::vector<AtTrack> modifiedTracks;

   // Iterate over the original tracks and store the modified ones.
   for (auto track : tracks)
      modifiedTracks.push_back(GetModifiedTrack(track, rawEvent, event));

   // Replace the vector of track candidates with the new modified one.
   patternEvent->SetTrackCand(modifiedTracks);
}
