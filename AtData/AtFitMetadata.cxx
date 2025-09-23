#include "AtFitMetadata.h"

ClassImp(AtFitMetadata);

AtFitMetadata::AtFitMetadata(const AtFitMetadata &copy) : fEventID(copy.fEventID)
{
   for (auto const& entry: copy.fMetadatas) {
      Int_t trackID = entry.first;
      TrackMetadatasVector newMetadatasVector;
      for (auto&& metadata: entry.second)
         newMetadatasVector.push_back(metadata->Clone());
      fMetadatas.insert({trackID, std::move(newMetadatasVector)});
   }
}
