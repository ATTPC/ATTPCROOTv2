#ifndef ATFITRESULT_H
#define ATFITRESULT_H

#include "AtFitTrackResult.h"

#include <FairLogger.h>

#include <Rtypes.h> // for Double_t, THashConsistencyHolder, ClassDefOverride
#include <TObject.h>

#include <functional>
#include <map>
#include <memory>
#include <utility>

class TBuffer;
class TClass;
class TMemberInspector;

/**
 * Class for storing the result of the fit for the entire AtTrackingEvent from an AtFitter class.
 */
class AtFitResult : public TObject {
public:
   using TrackResultPtr = std::unique_ptr<AtFitTrackResult>;
   using TrackResultsVector = std::vector<TrackResultPtr>;
   using ResultsMap = std::map<Int_t, TrackResultsVector>;

protected:
   // Vector to store the results for all different fits done to all tracks in the event.
   ResultsMap fResults;

   // Event ID for which this fit was done.
   ULong_t fEventID;

public:
   AtFitResult() = default;
   ~AtFitResult() = default;

   void SetTrackResultsVector(Int_t trackID, TrackResultsVector results) { fResults[trackID] = std::move(results); }

   void SetEventID(ULong_t id) { fEventID = id; }

   TrackResultsVector &GetTrackResultsVector(Int_t trackID) { return fResults.at(trackID); }

   ClassDefOverride(AtFitResult, 1);
};

#endif
