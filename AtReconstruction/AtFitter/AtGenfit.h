#ifndef AtGENFIT_H
#define AtGENFIT_H

#include "AtFitter.h"

#include "AtHit.h"
#include "AtHitCluster.h"
#include "AtSpacePointMeasurement.h"

// GENFIT2 classes
#include "AbsKalmanFitter.h"
#include "KalmanFitterRefTrack.h"
#include "KalmanFitter.h"
#include "DAF.h"
#include "ConstField.h"
#include "FieldManager.h"
#include "MaterialEffects.h"
#include "TGeoMaterialInterface.h"
#include "MeasurementFactory.h"
#include "MeasurementProducer.h"
#include "EventDisplay.h"
#include "TrackPoint.h"

//#include "GFRaveVertexFactory.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

namespace AtFITTER {

class AtGenfit : public AtFitter {

public:
   AtGenfit(Float_t magfield, Float_t minbrho, Float_t maxbrho, std::string eLossFile, Int_t minit = 5,
            Int_t maxit = 20);
   ~AtGenfit();

   genfit::Track *FitTracks(AtTrack *track);
   void Init();

   inline void SetMinIterations(Int_t minit) { fMinIterations = minit; }
   inline void SetMaxIterations(Int_t maxit) { fMaxIterations = maxit; }
   inline void SetMinBrho(Float_t minbrho) { fMinBrho = minbrho; }
   inline void SetMaxBrho(Float_t maxbrho) { fMaxBrho = maxbrho; }
   inline void SetMagneticField(Float_t magfield) { fMagneticField = 10.0 * magfield; }
   inline void SetPDGCode(Int_t pdgcode) { fPDGCode = pdgcode; }
   inline void SetMass(Float_t mass) { fMass = mass; }
   inline void SetAtomicNumber(Int_t znumber) { fAtomicNumber = znumber; }
   inline void SetNumFitPoints(Float_t points) { fNumFitPoints = points; }
   inline void SetVerbosityLevel(Int_t verbosity) { fVerbosity = verbosity; }
   inline void SetEnergyLossFile(std::string file) { fEnergyLossFile = file; }

   TClonesArray *GetGenfitTrackArray();

private:
   std::shared_ptr<genfit::AbsKalmanFitter> fKalmanFitter;
   TClonesArray *fGenfitTrackArray;
   TClonesArray *fHitClusterArray;
   Int_t fPDGCode; //<! Particle PGD code
   Int_t fTPCDetID;
   Int_t fCurrentDirection;
   Float_t fMaxBrho;            //<! Max Brho allowed in Tm
   Float_t fMinBrho;            //<! Min Brho allowed in Tm
   Int_t fMaxIterations;        //<! Max iterations for fitter
   Int_t fMinIterations;        //<! Min iterations for fitter
   Float_t fMagneticField;      //<! Constant magnetic field along Z in T
   Float_t fMass;               //<! Particle mass in atomic mass unit
   Int_t fAtomicNumber;         //<! Particle Atomic number Z
   Float_t fNumFitPoints;       //<! % of processed track points for fit
   Int_t fVerbosity;            //<! Fit verbosity
   std::string fEnergyLossFile; //<! Energy loss file
   Bool_t fSimulationConv;       //<! Switch to simulation convention
  
   genfit::MeasurementProducer<AtHitCluster, genfit::AtSpacepointMeasurement> *fMeasurementProducer;
   genfit::MeasurementFactory<genfit::AbsMeasurement> *fMeasurementFactory;

   std::vector<Int_t> *fPDGCandidateArray;

   ClassDef(AtGenfit, 1);
};

} // namespace AtFITTER

#endif
