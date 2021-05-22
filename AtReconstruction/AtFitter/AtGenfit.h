#ifndef AtGENFIT_H
#define AtGENFIT_H

#include "AtFitter.h"

#include "AtHit.h"
#include "AtHitCluster.h"
#include "AtSpacePointMeasurement.h"

// GENFIT2 classes
#include "AbsKalmanFitter.h"
#include "KalmanFitterRefTrack.h"
#include "DAF.h"
#include "ConstField.h"
#include "FieldManager.h"
#include "MaterialEffects.h"
#include "TGeoMaterialInterface.h"
#include "MeasurementFactory.h"
#include "MeasurementProducer.h"
#include "EventDisplay.h"

//#include "GFRaveVertexFactory.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

namespace AtFITTER {

class AtGenfit : public AtFitter {

public:
   AtGenfit();
  ~AtGenfit();

   bool FitTracks(AtPatternEvent &patternEvent);
   void Init();
   void SetMinIterations(Int_t value);
   void SetMaxIterations(Int_t value);
   TClonesArray* GetGenfitTrackArray();
  
  
private:
   genfit::AbsKalmanFitter *fKalmanFitter;
   TClonesArray *fGenfitTrackArray;
   TClonesArray *fHitClusterArray;
   Int_t fTPCDetID;
   Int_t fCurrentDirection;

   genfit::MeasurementProducer<AtHitCluster, genfit::AtSpacepointMeasurement> *fMeasurementProducer;
   genfit::MeasurementFactory<genfit::AbsMeasurement> *fMeasurementFactory;

   std::vector<Int_t> *fPDGCandidateArray;

   ClassDef(AtGenfit, 1);
};

} // namespace AtFITTER

#endif
