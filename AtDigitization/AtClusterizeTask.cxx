#include "AtClusterizeTask.h"

// Fair class header
#include "FairRootManager.h"
#include "FairRunAna.h"
#include "FairRuntimeDb.h"

#include "AtDigiPar.h"
#include "AtGas.h"
#include "AtSimulatedPoint.h"
#include "AtMCPoint.h"
#include "AtVertexPropagator.h"

// STL class headers
#include <cmath>
#include <iostream>
#include <iomanip>

#include "TClonesArray.h"
#include "TF1.h"
#include "TMath.h"
#include "TRandom.h"

using XYZVector = ROOT::Math::XYZVector;

AtClusterizeTask::AtClusterizeTask() : FairTask("AtClusterizeTask"), fEventID(0), fIsPersistent(kFALSE) {}

AtClusterizeTask::AtClusterizeTask(const char *name) : FairTask(name), fEventID(0), fIsPersistent(kFALSE) {}

AtClusterizeTask::~AtClusterizeTask()
{
   LOG(debug) << "Destructor of AtClusterizeTask";
}

void AtClusterizeTask::SetParContainers()
{
   LOG(debug) << "SetParContainers of AtClusterizeTask";

   FairRunAna *ana = FairRunAna::Instance();
   FairRuntimeDb *rtdb = ana->GetRuntimeDb();
   fPar = dynamic_cast<AtDigiPar *>(rtdb->getContainer("AtDigiPar"));
   if (fPar == nullptr)
      LOG(fatal) << "Could not get the parameter container "
                 << "AtDigiPar";
}

void AtClusterizeTask::getParameters()
{
   fEIonize = fPar->GetEIonize() / 1000000; // [MeV]
   fFano = fPar->GetFano();
   fVelDrift = fPar->GetDriftVelocity();   // [cm/us]
   fCoefT = fPar->GetCoefDiffusionTrans(); // [cm^2/us]
   fCoefL = fPar->GetCoefDiffusionLong();  // [cm^2/us]

   fDetPadPlane = fPar->GetZPadPlane(); //[mm]

   std::cout << "  Ionization energy of gas: " << fEIonize << " MeV" << std::endl;
   std::cout << "  Fano factor of gas: " << fFano << std::endl;
   std::cout << "  Drift velocity: " << fVelDrift << std::endl;
   std::cout << "  Longitudal coefficient of diffusion: " << fCoefT << std::endl;
   std::cout << "  Transverse coefficient of diffusion: " << fCoefL << std::endl;
   std::cout << "  Position of the pad plane (Z): " << fDetPadPlane << std::endl;
}

InitStatus AtClusterizeTask::Init()
{
   LOG(debug) << "Initilization of AtClusterizeTask";

   FairRootManager *ioman = FairRootManager::Instance();

   fMCPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtTpcPoint"));
   if (fMCPointArray == nullptr) {
      LOG(error) << "Cannot find fMCPointArray array!";
      return kERROR;
   }

   fSimulatedPointArray = new TClonesArray("AtSimulatedPoint");
   ioman->Register("AtSimulatedPoint", "cbmsim", fSimulatedPointArray, fIsPersistent);

   getParameters();

   return kSUCCESS;
}

void AtClusterizeTask::setNewTrack()
{
   fPrevPoint = getCurrentPointLocation();
   fCurrTrackID = fMCPoint->GetTrackID();
}

void AtClusterizeTask::processPoint(Int_t mcPointID)
{
   auto trackID = fMCPoint->GetTrackID();

   // If it is a new track entering the volume or no energy was deposited
   // record its location and the new track ID
   if (fMCPoint->GetEnergyLoss() == 0 || fCurrTrackID != trackID) {
      setNewTrack();
      std::cout << "Point is new track" << std::endl;
      return;
   }

   XYZVector currentPoint = getCurrentPointLocation(); // [mm, mm, us]
   auto genElectrons = getNumberOfElectronsGenerated();

   XYZVector step;
   if (genElectrons > 0) {
      step = (currentPoint - fPrevPoint) / genElectrons;
   }

   auto sigTrans = getTransverseDiffusion(currentPoint.z());  // mm
   auto sigLong = getLongitudinalDiffusion(currentPoint.z()); // us

   // Need to loop through electrons
   for (int i = 0; i < genElectrons; ++i) {
      auto loc = applyDiffusion(currentPoint + i * step, sigTrans, sigLong);
      auto size = fSimulatedPointArray->GetEntriesFast();
      AtSimulatedPoint *simElec = new ((*fSimulatedPointArray)[size]) AtSimulatedPoint(mcPointID, i, loc);
   }

   fPrevPoint = currentPoint;
}

void AtClusterizeTask::Exec(Option_t *option)
{
   fSimulatedPointArray->Delete();

   for (int i = 0; i < fMCPointArray->GetEntries(); ++i) {
      fMCPoint = dynamic_cast<AtMCPoint *>(fMCPointArray->At(i));
      if (fMCPoint->GetVolName() == "drift_volume")
         processPoint(i);
      else
         LOG(info) << "Skipping point " << i << ". Not in drift volume.";
   }
}

// Takes drift time in us
Double_t AtClusterizeTask::getLongitudinalDiffusion(Double_t driftTime)
{
   auto sigInMM = TMath::Sqrt(10. * fCoefL * 2 * driftTime);
   auto sigInUs = sigInMM / fVelDrift;
   return sigInUs;
}

// Takes drift time in us
Double_t AtClusterizeTask::getTransverseDiffusion(Double_t driftTime)
{
   return TMath::Sqrt(10. * fCoefT * 2 * driftTime);
}

UInt_t AtClusterizeTask::getNumberOfElectronsGenerated()
{
   auto energyLoss = fMCPoint->GetEnergyLoss() * 1000.;
   auto meanElec = energyLoss / fEIonize;
   auto sigElec = TMath::Sqrt(fFano * meanElec);
   return gRandom->Gaus(meanElec, sigElec);
}

XYZVector AtClusterizeTask::getCurrentPointLocation()
{
   auto zInCm = fDetPadPlane / 10. - fMCPoint->GetZIn();
   auto driftTime = TMath::Abs(zInCm) / fVelDrift; // us

   return XYZVector(fMCPoint->GetXIn() * 10., fMCPoint->GetYIn() * 10., driftTime);
}

ROOT::Math::XYZVector
AtClusterizeTask::applyDiffusion(const ROOT::Math::XYZVector &loc, double_t sigTrans, double sigLong)
{
   auto r = gRandom->Gaus(0, sigTrans);
   auto phi = gRandom->Uniform(0, TMath::TwoPi());
   auto dz = gRandom->Gaus(0, sigLong);

   return loc + XYZVector(r * TMath::Cos(phi), r * TMath::Sin(phi), dz);
}

ClassImp(AtClusterizeTask);
