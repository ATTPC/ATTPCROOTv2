#ifndef AtBRAGGCURVEFITTER_H
#define AtBRAGGCURVEFITTER_H

#include "AtFittedTrack.h"
#include "AtFitter.h"

namespace EventFit {

class AtBraggCurveFitter : public AtFitter {
public:
   using TrackMetadataPtr = std::unique_ptr<AtFitTrackMetadata>;
   using TrackMetadatasVector = std::vector<TrackMetadataPtr>;
   using TrackMetadatasSet =
      std::set<TrackMetadataPtr, std::function<bool(const TrackMetadataPtr &, const TrackMetadataPtr &)>>;

public:
   AtBraggCurveFitter() = default;
   ~AtBraggCurveFitter() = default;

   virtual void Init() override;

protected:
   virtual AtFittedTrack *GetFittedTrack(AtTrack *track, AtFitMetadata *fitMetadata = nullptr,
                                         AtRawEvent *rawEvent = nullptr, AtEvent *event = nullptr) override;

   // Compare function that will be used to sort the fit results for a given track.
   virtual bool
   CompareTrackFitsFunction(const TrackMetadataPtr &trackMetadataA, const TrackMetadataPtr &trackMetadataB) override;

   ClassDefOverride(AtBraggCurveFitter, 1);
};

} // namespace EventFit

#endif
