#ifndef ATGENFIT_H
#define ATGENFIT_H

#include "ATFitter.hh"

#include "ATHit.hh"
#include "ATHitCluster.hh"
#include "ATSpacePointMeasurement.hh"

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

namespace ATFITTER{


class ATGenfit : public ATFitter
{

  public:
      ATGenfit();
      ~ATGenfit();

      bool FitTracks(ATPatternEvent &patternEvent);
      void Init();
      void SetMinIterations(Int_t value);
      void SetMaxIterations(Int_t value);

  private:

      genfit::AbsKalmanFitter *fKalmanFitter;
      TClonesArray *fGenfitTrackArray;
      TClonesArray *fHitClusterArray;
      Int_t fTPCDetID;
      Int_t fCurrentDirection;

      genfit::MeasurementProducer<ATHitCluster, genfit::ATSpacepointMeasurement> *fMeasurementProducer;
      genfit::MeasurementFactory<genfit::AbsMeasurement> *fMeasurementFactory;

      std::vector<Int_t> *fPDGCandidateArray;

  ClassDef(ATGenfit, 1);

};

}//namespace

#endif

