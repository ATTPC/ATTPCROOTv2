#ifndef ATGENFIT_H
#define ATGENFIT_H

#include "AtFitter.h"

#include <Rtypes.h>
#include <TMath.h> // for DegToRad
#include <Track.h>

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
   Int_t fCurrentDirection{-1};
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

   genfit::MeasurementProducer<AtHitCluster, genfit::AtSpacepointMeasurement> *fMeasurementProducer;
   genfit::MeasurementFactory<genfit::AbsMeasurement> *fMeasurementFactory;

   std::vector<Int_t> *fPDGCandidateArray{};

public:
   AtGenfit(Float_t magfield, Float_t minbrho, Float_t maxbrho, std::string eLossFile, Float_t gasMediumDensity,
            Int_t pdg = 2212, Int_t minit = 5, Int_t maxit = 20);
   ~AtGenfit();

   genfit::Track *FitTracks(AtTrack *track) override;
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

   TClonesArray *GetGenfitTrackArray();
   Int_t GetPDGCode() { return fPDGCode; }
   std::string &GetIonName() { return fIonName; }

protected:
   inline bool IsForwardTrack(double theta) { return theta < 90.0 * TMath::DegToRad(); }
   ClassDefOverride(AtGenfit, 1);
};

} // namespace AtFITTER

#endif
