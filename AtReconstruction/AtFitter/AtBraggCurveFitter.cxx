#include "AtBraggCurveFitter.h"

ClassImp(EventFit::AtBraggCurveFitter);

void EventFit::AtBraggCurveFitter::Init() {}

AtFittedTrack *EventFit::AtBraggCurveFitter::GetFittedTrack(AtTrack *track, AtFitMetadata *fitMetadata,
                                                            AtRawEvent *rawEvent, AtEvent *event)
{

   return new AtFittedTrack();
}

bool EventFit::AtBraggCurveFitter::CompareTrackFitsFunction(const TrackMetadataPtr &trackMetadataA,
                                                            const TrackMetadataPtr &trackMetadataB)
{

   return true;
}
