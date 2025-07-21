#ifndef AtFITTER_H
#define AtFITTER_H

#include "AtFitResult.h"

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
class AtPatternEvent;

namespace AtFITTER {

class AtFitter : public TObject {
public:
   using TrackResultPtr = std::unique_ptr<AtFitTrackResult>;
   using TrackResultsVector = std::vector<TrackResultPtr>;
   using TrackResultsSet =
      std::set<TrackResultPtr, std::function<bool(const TrackResultPtr &, const TrackResultPtr &)>>;

protected:
   // Pointer to the AtPatternEvent to be fitted.
   AtPatternEvent *fPatternEvent{nullptr};

   // Pointer to the AtFitResult object in which store the fit metadata.
   AtFitResult *fFitResult{nullptr};

   // Pointers to AtRawEvent and AtEvent. In case some specific fitter needs to access information
   // in any of those branches.
   AtRawEvent *fRawEvent{nullptr};
   AtEvent *fEvent{nullptr};

   // Compare function that will be used to sort the fit results for a given track.
   virtual bool CompareTrackFitsFunction(const TrackResultPtr &trackResultA, const TrackResultPtr &trackResultB) = 0;

public:
   AtFitter() = default;
   ~AtFitter() = default;

   virtual std::vector<std::unique_ptr<AtFittedTrack>> ProcessEvent() = 0;
   virtual void Init() = 0;

   // Mandatory to set.
   void SetPatternEvent(AtPatternEvent *patternEvent) { fPatternEvent = patternEvent; }
   void SetFitResult(AtFitResult *fitResult) { fFitResult = fitResult; }

   // Optional to set.
   void SetRawEvent(AtRawEvent *rawEvent) { fRawEvent = rawEvent; }
   void SetEvent(AtEvent *event) { fEvent = event; }

   // Reset pointers to AtPatternEvent, AtRawEvent and AtEvent.
   void Reset();

protected:
   ClassDef(AtFitter, 2);
};

} // namespace AtFITTER

#endif
