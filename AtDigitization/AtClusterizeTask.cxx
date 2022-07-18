#include "AtClusterizeTask.h"

#include <FairLogger.h>
#include <FairParSet.h>
#include <FairTask.h>

#include <TMathBase.h>
#include <TObject.h>
#include <TString.h>

#include <iostream>
#include <memory>

// Fair class header
#include "AtDigiPar.h"
#include "AtMCPoint.h"
#include "AtSimulatedPoint.h"

#include <FairRootManager.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>

#include <TClonesArray.h>
#include <TMath.h>
#include <TRandom.h>

using XYZVector = ROOT::Math::XYZVector;

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

AtClusterizeTask::AtClusterizeTask() : AtClusterizeTask("AtClusterizeTask") {}

AtClusterizeTask::AtClusterizeTask(const char *name) : FairTask(name) {}

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
   LOG(INFO) << "Initilization of AtClusterizeTask";

   FairRootManager *ioman = FairRootManager::Instance();

   fMCPointArray = dynamic_cast<TClonesArray *>(ioman->GetObject("AtTpcPoint"));
   if (fMCPointArray == nullptr) {
      LOG(error) << "Cannot find fMCPointArray array!";
      return kERROR;
   }

   fSimulatedPointArray = std::make_unique<TClonesArray>("AtSimulatedPoint");
   ioman->Register("AtSimulatedPoint", "cbmsim", fSimulatedPointArray.get(), fIsPersistent);

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
      new ((*fSimulatedPointArray)[size]) AtSimulatedPoint(mcPointID, i, loc);
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
   auto sigInMM = TMath::Sqrt(100. * fCoefL * 2 * driftTime);
   auto sigInUs = sigInMM / fVelDrift;
   return sigInUs;
}

// Takes drift time in us
Double_t AtClusterizeTask::getTransverseDiffusion(Double_t driftTime)
{
   return TMath::Sqrt(100. * fCoefT * 2 * driftTime);
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
   auto zInCm = fDetPadPlane / 10. - fMCPoint->GetZ();
   auto driftTime = TMath::Abs(zInCm) / fVelDrift; // us

   return {fMCPoint->GetX() * 10., fMCPoint->GetY() * 10., driftTime};
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
