#ifndef AtFITTER_H
#define AtFITTER_H

#include "AtFitMetadata.h"

#include <Rtypes.h>
#include <TObject.h>

#include <functional>
#include <memory>
#include <set>
#include <tuple>
#include <vector>

class AtDigiPar;
class AtTrack;
class AtFittedTrack;
class TBuffer;
class TClass;
class TMemberInspector;
class AtRawEvent;
class AtEvent;
class AtFitMetadata;
class AtPatternEvent;
class AtRawEvent;
class AtTrackingEvent;

namespace EventFit {

class AtFitter : public TObject {
public:
   using TrackMetadataPtr = std::unique_ptr<AtFitTrackMetadata>;
   using TrackMetadatasVector = std::vector<TrackMetadataPtr>;
   using TrackMetadatasSet =
      std::set<TrackMetadataPtr, std::function<bool(const TrackMetadataPtr &, const TrackMetadataPtr &)>>;

public:
   AtFitter() = default;
   ~AtFitter() = default;

   virtual void FitEvent(AtTrackingEvent *trackingEvent, AtPatternEvent *patternEvent,
                         AtFitMetadata *fitMetadata = nullptr, AtRawEvent *rawEvent = nullptr,
                         AtEvent *event = nullptr);
   virtual void Init() = 0;

protected:
   virtual AtFittedTrack *GetFittedTrack(AtTrack *track, AtFitMetadata *fitMetadata = nullptr,
                                         AtRawEvent *rawEvent = nullptr, AtEvent *event = nullptr) = 0;

   // Compare function that will be used to sort the fit results for a given track.
   virtual bool
   CompareTrackFitsFunction(const TrackMetadataPtr &trackMetadataA, const TrackMetadataPtr &trackMetadataB) = 0;

   ClassDef(AtFitter, 2);
};

} // namespace EventFit

#endif
