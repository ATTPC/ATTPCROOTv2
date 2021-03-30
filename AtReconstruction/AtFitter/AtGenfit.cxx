#include "AtGenfit.h"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(AtFITTER::AtGenfit)

   AtFITTER::AtGenfit::AtGenfit()
{

   fTPCDetID = 0;
   fCurrentDirection = 1;

   fKalmanFitter = new genfit::KalmanFitterRefTrack();
   fKalmanFitter->setMinIterations(5);
   fKalmanFitter->setMaxIterations(20);

   fGenfitTrackArray = new TClonesArray("genfit::Track");
   fHitClusterArray = new TClonesArray("AtHitCluster");

   fMeasurementProducer =
      new genfit::MeasurementProducer<AtHitCluster, genfit::AtSpacepointMeasurement>(fHitClusterArray);
   fMeasurementFactory = new genfit::MeasurementFactory<genfit::AbsMeasurement>();
   fMeasurementFactory->addProducer(fTPCDetID, fMeasurementProducer);

   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0., 20.)); // TODO kGauss
   genfit::MaterialEffects *materialEffects = genfit::MaterialEffects::getInstance();
   materialEffects->init(new genfit::TGeoMaterialInterface());

   fPDGCandidateArray = new std::vector<Int_t>; // TODO
   fPDGCandidateArray->push_back(2212);
}

AtFITTER::AtGenfit::~AtGenfit() {}

void AtFITTER::AtGenfit::Init()
{
   fHitClusterArray->Delete();
   fGenfitTrackArray->Delete();
}

void AtFITTER::AtGenfit::SetMinIterations(Int_t value)
{
   fKalmanFitter->setMinIterations(value);
}
void AtFITTER::AtGenfit::SetMaxIterations(Int_t value)
{
   fKalmanFitter->setMaxIterations(value);
}

bool AtFITTER::AtGenfit::FitTracks(AtPatternEvent &patternEvent)
{

   fHitClusterArray->Delete();
   genfit::TrackCand trackCand;

   std::vector<AtTrack> &patternTrackCand = patternEvent.GetTrackCand();

   for (auto iTrack = 0; iTrack < patternTrackCand.size(); ++iTrack) {

   } // iTrack
}
