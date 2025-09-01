#ifndef ATFITMETADATA_H
#define ATFITMETADATA_H

#include "AtFitTrackMetadata.h"

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
class AtFitMetadata : public TObject {
public:
   using TrackMetadataPtr = std::unique_ptr<AtFitTrackMetadata>;
   using TrackMetadatasVector = std::vector<TrackMetadataPtr>;
   using MetadatasMap = std::map<Int_t, TrackMetadatasVector>;

protected:
   /**
    * Map to store the metadatas for all different fits done to all tracks in the event.
    * The Int_t corresponds to the trackID for which the metadatas correspond to.
    * The vector of AtFitTrackMetadata contains the different metadatas for all fits
    * that have been done for the track (for example, different assumptions for the
    * particles of the track, different initial conditions, etc...).
    */
   MetadatasMap fMetadatas;

   // Event ID for which this fit was done.
   ULong_t fEventID;

public:
   AtFitMetadata() = default;
   ~AtFitMetadata() = default;

   void SetTrackMetadatasVector(Int_t trackID, TrackMetadatasVector metadatas)
   {
      fMetadatas[trackID] = std::move(metadatas);
   }

   void SetEventID(ULong_t id) { fEventID = id; }

   TrackMetadatasVector &GetTrackMetadatasVector(Int_t trackID) { return fMetadatas.at(trackID); }

   ClassDefOverride(AtFitMetadata, 1);
};

#endif
