#ifndef AtFITTER_H
#define AtFITTER_H

#include "AtTrackTransformer.h"

#include <Rtypes.h>
#include <TObject.h>

#include <memory>
#include <tuple>
#include <vector>

class AtDigiPar;
class AtTrack;
class FairLogger;
class TBuffer;
class TClass;
class TMemberInspector;

namespace genfit {
class Track;
} // namespace genfit

namespace AtFITTER {

class AtFitter : public TObject {

public:
   AtFitter();
   virtual ~AtFitter();
   // virtual std::vector<AtTrack> GetFittedTrack() = 0;
   virtual genfit::Track *FitTracks(AtTrack *track) = 0;
   virtual void Init() = 0;

   void MergeTracks(std::vector<AtTrack> *trackCandSource, std::vector<AtTrack> *trackJunkSource,
                    std::vector<AtTrack> *trackDest, bool fitDirection, bool simulationConv);
   Bool_t MergeTracks(std::vector<AtTrack *> *trackCandSource, std::vector<AtTrack> *trackDest,
                      Bool_t enableSingleVertexTrack, Double_t clusterRadius, Double_t clusterDistance);

protected:
   FairLogger *fLogger{}; ///< logger pointer
   AtDigiPar *fPar{};     ///< parameter container
   std::unique_ptr<AtTools::AtTrackTransformer> fTrackTransformer{std::make_unique<AtTools::AtTrackTransformer>()};
   std::tuple<Double_t, Double_t>
   GetMomFromBrho(Double_t A, Double_t Z,
                  Double_t brho);                      ///< Returns momentum (in GeV) from Brho assuming M (amu) and Z;
   Bool_t FindVertexTrack(AtTrack *trA, AtTrack *trB); ///< Lambda function to find track closer to vertex
   ClassDef(AtFitter, 1);
};

} // namespace AtFITTER

#endif
