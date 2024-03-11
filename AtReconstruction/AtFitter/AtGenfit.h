#ifndef ATGENFIT_H
#define ATGENFIT_H

#include "AtFitter.h"
#include "AtFormat.h"
#include "AtKinematics.h"
#include "AtParsers.h"

#include <Rtypes.h>
#include <TMath.h> // for DegToRad
#include <Track.h>

#include "AbsFitterInfo.h"
#include "AbsKalmanFitter.h"
#include "ConstField.h"
#include "DAF.h"
#include "EventDisplay.h"
#include "Exception.h"
#include "FairLogger.h"
#include "FairRootManager.h"
#include "FairRun.h"
#include "FairRunAna.h"
#include "FieldManager.h"
#include "FitStatus.h"
#include "KalmanFitStatus.h"
#include "KalmanFitterInfo.h"
#include "KalmanFitterRefTrack.h"
#include "MaterialEffects.h"
#include "MeasuredStateOnPlane.h"
#include "MeasurementFactory.h"
#include "MeasurementOnPlane.h"
#include "MeasurementProducer.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

class AtHitCluster;
class AtTrack;
class TBuffer;
class TClass;
class TClonesArray;
class TMemberInspector;

namespace genfit {
class AbsKalmanFitter;
class AbsMeasurement;
class AtSpacepointMeasurement;
template <class hit_T, class measurement_T>
class MeasurementProducer;
template <class measurement_T>
class MeasurementFactory;
} // namespace genfit

namespace AtFITTER {

class AtGenfit : public AtFitter {
private:
   std::shared_ptr<genfit::AbsKalmanFitter> fKalmanFitter;
   TClonesArray *fGenfitTrackArray;
   TClonesArray *fHitClusterArray;
   Int_t fPDGCode{2212}; //<! Particle PGD code
   Int_t fTPCDetID{0};
   Int_t fFitDirection{0};
   Float_t fMaxBrho;              //<! Max Brho allowed in Tm
   Float_t fMinBrho;              //<! Min Brho allowed in Tm
   Int_t fMaxIterations;          //<! Max iterations for fitter
   Int_t fMinIterations;          //<! Min iterations for fitter
   Float_t fMagneticField;        //<! Constant magnetic field along Z in T
   Float_t fMass{1.00727647};     //<! Particle mass in atomic mass unit
   Int_t fAtomicNumber{1};        //<! Particle Atomic number Z
   Float_t fNumFitPoints{0.90};   //<! % of processed track points for fit
   Int_t fVerbosity{0};           //<! Fit verbosity
   std::string fEnergyLossFile;   //<! Energy loss file
   Bool_t fSimulationConv{false}; //<! Switch to simulation convention
   Float_t fGasMediumDensity{};   //<! Medium density in mg/cm3
   Double_t fPhiOrientation{0};   //<! Phi angle orientation for fit
   std::string fIonName;          //<! Name of ion to fit
   Bool_t fNoMaterialEffects;     //<! Disable material effects in GENFIT
   Bool_t fEnableMerging{0};
   Bool_t fEnableSingleVertexTrack{0};
   Bool_t fEnableReclustering{0};
   Double_t fClusterSize{0};
   Double_t fClusterRadius{0};

   std::vector<AtTools::IonFitInfo> *ionList;
   std::unique_ptr<AtTools::AtTrackTransformer> fTrackTransformer;
   std::shared_ptr<AtTools::AtKinematics> fKinematics;

   genfit::MeasurementProducer<AtHitCluster, genfit::AtSpacepointMeasurement> *fMeasurementProducer;
   genfit::MeasurementFactory<genfit::AbsMeasurement> *fMeasurementFactory;

   std::vector<Int_t> *fPDGCandidateArray{};

   std::vector<AtTrack *> FindSingleTracks(std::vector<AtTrack *> &tracks);
   Double_t CenterDistance(AtTrack *trA, AtTrack *trB);
   Bool_t CompareTracks(AtTrack *trA, AtTrack *trB);
   Bool_t CheckOverlap(AtTrack *trA, AtTrack *trB);

public:
   AtGenfit(Float_t magfield, Float_t minbrho, Float_t maxbrho, std::string eLossFile, Float_t gasMediumDensity,
            Int_t pdg = 2212, Int_t minit = 5, Int_t maxit = 20, Bool_t noMatEffects = kFALSE);
   ~AtGenfit();

   enum Exp { e20020, e20009, a1954, a1975, a1954b };
   Exp fExpNum{a1975};

   genfit::Track *FitTracks(AtTrack *track);
   std::vector<std::unique_ptr<AtFittedTrack>> ProcessTracks(std::vector<AtTrack> &tracks) override;
   void Init() override;

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
   inline void SetSimulationConvention(Bool_t simconv) { fSimulationConv = simconv; }
   inline void SetGasMediumDensity(Float_t mediumDensity) { fGasMediumDensity = mediumDensity; }
   inline void RotatePhi(Double_t phi) { fPhiOrientation = phi; }
   inline void SetIonName(std::string ionName) { fIonName = std::move(ionName); }
   inline void EnableMerging(Bool_t merging) { fEnableMerging = merging; }
   inline void EnableSingleVertexTrack(Bool_t singletrack) { fEnableSingleVertexTrack = singletrack; }
   inline void EnableReclustering(Bool_t reclustering, Double_t clusterRadius, Double_t clusterSize)
   {
      fEnableReclustering = reclustering;
      fClusterRadius = clusterRadius;
      fClusterSize = clusterSize;
   }

   inline void SetExpNum(Exp exp) { fExpNum = exp; }
   inline void SetFitDirection(Int_t direction) { fFitDirection = direction; }

   TClonesArray *GetGenfitTrackArray();
   Int_t GetPDGCode() { return fPDGCode; }
   std::string &GetIonName() { return fIonName; }

protected:
   inline bool IsForwardTrack(double theta) { return theta < 90.0 * TMath::DegToRad(); }
   ClassDefOverride(AtGenfit, 1);
};

} // namespace AtFITTER

#endif
