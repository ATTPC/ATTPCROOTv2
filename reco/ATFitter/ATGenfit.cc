#include "ATGenfit.hh"

// FairRoot classes
#include "FairRuntimeDb.h"
#include "FairRun.h"

ClassImp(ATFITTER::ATGenfit)

ATFITTER::ATGenfit::ATGenfit()
{

  fTPCDetID = 0;
  fCurrentDirection = 1;

  fKalmanFitter = new genfit::KalmanFitterRefTrack();
  fKalmanFitter -> setMinIterations(5);
  fKalmanFitter -> setMaxIterations(20);

  fGenfitTrackArray = new TClonesArray("genfit::Track");
  fHitClusterArray = new TClonesArray("ATHitCluster");

  fMeasurementProducer = new genfit::MeasurementProducer<ATHitCluster, genfit::ATSpacepointMeasurement>(fHitClusterArray);
  fMeasurementFactory = new genfit::MeasurementFactory<genfit::AbsMeasurement>();
  fMeasurementFactory -> addProducer(fTPCDetID, fMeasurementProducer);

  genfit::FieldManager::getInstance() -> init(new genfit::ConstField(0., 0., 20.)); //TODO kGauss
  genfit::MaterialEffects *materialEffects = genfit::MaterialEffects::getInstance();
  materialEffects -> init(new genfit::TGeoMaterialInterface());

  fPDGCandidateArray = new std::vector<Int_t>;//TODO
  fPDGCandidateArray -> push_back(2212);
   

}

ATFITTER::ATGenfit::~ATGenfit()
{

}

void ATFITTER::ATGenfit::Init()
{
  fHitClusterArray -> Delete();
  fGenfitTrackArray -> Delete();
}

void ATFITTER::ATGenfit::SetMinIterations(Int_t value) { 
  fKalmanFitter -> setMinIterations(value);
}
void ATFITTER::ATGenfit::SetMaxIterations(Int_t value) { 
  fKalmanFitter -> setMaxIterations(value);
}

bool ATFITTER::ATGenfit::FitTracks(ATPatternEvent &patternEvent)
{

  fHitClusterArray -> Delete();
  genfit::TrackCand trackCand;
  
  std::vector<ATTrack>& patternTrackCand =  patternEvent.GetTrackCand();

  for(auto iTrack = 0;iTrack<patternTrackCand.size();++iTrack)
  {


  }//iTrack
 


}
