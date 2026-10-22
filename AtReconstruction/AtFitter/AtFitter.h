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
   AtFitter() = default;
   ~AtFitter() = default;

   virtual void FitEvent(AtTrackingEvent *trackingEvent, AtPatternEvent *patternEvent,
                         AtFitMetadata *fitMetadata = nullptr, AtRawEvent *rawEvent = nullptr,
                         AtEvent *event = nullptr);
   virtual void Init() = 0;

protected:
   virtual AtFittedTrack *GetFittedTrack(AtTrack *track, AtFitMetadata *fitMetadata = nullptr,
                                         AtRawEvent *rawEvent = nullptr, AtEvent *event = nullptr) = 0;
};

} // namespace EventFit

#endif
