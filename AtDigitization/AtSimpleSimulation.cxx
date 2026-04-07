
#include "AtSimpleSimulation.h"

#include "AtELossModel.h"
#include "AtKinematics.h"
#include "AtMCPoint.h"
#include "AtPropagator.h"
#include "AtSpaceChargeModel.h" // for AtSpaceChargeModel

#include <FairLogger.h>
#include <FairRootManager.h>

#include <TClonesArray.h> // for TClonesArray
#include <TGeoManager.h>
#include <TGeoNavigator.h>
#include <TGeoNode.h>
#include <TGeoVolume.h>
#include <TObject.h> // for TObject

#include <cmath>     // for sqrt
#include <stdexcept> // for invalid_argument
#include <utility>   // for pair

thread_local TClonesArray AtSimpleSimulation::fMCPoints("AtMCPoint");
thread_local int AtSimpleSimulation::fTrackID = 0;

using SpaceChargeModel = std::shared_ptr<AtSpaceChargeModel>;
using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;
using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

namespace {
constexpr int kMaxCurvedTransportSteps = 200000;
constexpr int kMaxMinStepCurvedSteps = 4096;
constexpr double kMinStepGuardScale = 1.01;
}

// ---------------------------------------------------------------------------
// Thin wrapper so a shared_ptr<AtELossModel> can be passed to AtPropagator
// (which requires a unique_ptr<AtELossModel>).
// ---------------------------------------------------------------------------
namespace {
class ELossModelShared : public AtTools::AtELossModel {
   std::shared_ptr<AtTools::AtELossModel> fImpl;

public:
   explicit ELossModelShared(std::shared_ptr<AtTools::AtELossModel> impl)
      : AtTools::AtELossModel(0), fImpl(std::move(impl))
   {
   }
   double GetdEdx(double e) const override { return fImpl->GetdEdx(e); }
   double GetRange(double ei, double ef = 0) const override { return fImpl->GetRange(ei, ef); }
   double GetEnergyLoss(double ei, double d) const override { return fImpl->GetEnergyLoss(ei, d); }
   double GetEnergy(double ei, double d) const override { return fImpl->GetEnergy(ei, d); }
   double GetElossStraggling(double ei, double ef) const override { return fImpl->GetElossStraggling(ei, ef); }
   double GetdEdxStraggling(double ei, double ef) const override { return fImpl->GetdEdxStraggling(ei, ef); }
   double GetRangeVariance(double e) const override { return fImpl->GetRangeVariance(e); }
};

int GetPDGFromZA(int Z, int A)
{
   if (A == 1 && Z == 1)
      return 2212;
   if (A == 1 && Z == 0)
      return 2112;
   return 1000000000 + Z * 10000 + A * 10;
}
} // namespace

AtSimpleSimulation::AtSimpleSimulation(std::string geoFile)
{
   TGeoManager *geo = TGeoManager::Import(geoFile.c_str());

   if (gGeoManager == nullptr)
      LOG(fatal) << "Failed to load geometry file " << geoFile << " " << geo;

   fGeoManager = gGeoManager;
   fNavigator = nullptr;
}
AtSimpleSimulation::AtSimpleSimulation()
{
   // Defer geometry check until first use. FairRunSim::Init() sets up
   // gGeoManager, which may not have happened yet at construction time.
   // GetVolume() re-syncs with gGeoManager on each call.
   fGeoManager = nullptr;
   fNavigator = nullptr;
}

bool AtSimpleSimulation::ParticleID::operator<(const ParticleID &other) const
{
   if (A < other.A) {
      return true;
   } else if (A > other.A) {
      return false;
   } else {
      return Z < other.Z;
   }
}

/// Takes position in mm
TGeoVolume *AtSimpleSimulation::GetVolume(const XYZPoint &point)
{
   auto pointCm = point / 10.;
   {
      std::lock_guard<std::mutex> lock(fGeoMutex);
      if (gGeoManager == nullptr)
         return nullptr;

      if (fGeoManager != gGeoManager || fNavigator == nullptr) {
         fGeoManager = gGeoManager;
         fNavigator = fGeoManager->AddNavigator();
      }

      TGeoNode *node = fNavigator->FindNode(pointCm.X(), pointCm.Y(), pointCm.Z());
      if (node == nullptr) {
         return nullptr;
      }
      return node->GetVolume();
   }
}

bool AtSimpleSimulation::IsInVolume(const std::string &volName, const XYZPoint &point)
{

   TGeoVolume *volume = GetVolume(point);
   if (volume == nullptr || volName != std::string(volume->GetName())) {
      return false;
   }

   return true;
}

std::string AtSimpleSimulation::GetVolumeName(const XYZPoint &point)
{
   TGeoVolume *volume = GetVolume(point);
   if (volume == nullptr) {
      return "";
   }
   return volume->GetName();
}

void AtSimpleSimulation::AddModel(int Z, int A, ModelPtr model)
{
   AddModel(Z, A, model, static_cast<double>(A));
}

void AtSimpleSimulation::AddModel(int Z, int A, ModelPtr model, double massAmu)
{
   static constexpr double kEperAMU = 931.494; // MeV/c² per amu
   static constexpr double kEcharge = 1.602176634e-19; // Coulombs

   ParticleID id = {.A = A, .Z = Z};
   fModels[id] = {model, Z * kEcharge, massAmu * kEperAMU};
}

std::pair<XYZPoint, PxPyPzEVector>
AtSimpleSimulation::SimulateParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                     std::function<bool(XYZPoint, PxPyPzEVector)> func)
{
   auto modelIt = fModels.find({A, Z});
   if (modelIt == fModels.end())
      throw std::invalid_argument("Missing energy loss model for Z:" + std::to_string(Z) + " A:" + std::to_string(A));
   if (!IsInVolume("drift_volume", iniPos))
      throw std::invalid_argument("Position of particle is not in active volume but is in " + GetVolumeName(iniPos));

   return SimulateParticle(modelIt->second, iniPos, iniMom, func);
}

std::pair<XYZPoint, PxPyPzEVector>
AtSimpleSimulation::TransportParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                      StepCallback callback)
{
   auto modelIt = fModels.find({A, Z});
   if (modelIt == fModels.end())
      throw std::invalid_argument("Missing energy loss model for Z:" + std::to_string(Z) + " A:" + std::to_string(A));
   if (GetVolume(iniPos) == nullptr)
      throw std::invalid_argument("Position of particle is outside the loaded geometry");

   return TransportParticle(modelIt->second, GetPDGFromZA(Z, A), iniPos, iniMom, callback);
}

std::pair<XYZPoint, PxPyPzEVector>
AtSimpleSimulation::SimulateParticle(const ParticleInfo &info, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                     std::function<bool(XYZPoint, PxPyPzEVector)> func)
{
   // This is a new track
   fTrackID++;

   // -----------------------------------------------------------------------
   // Curved-track path: use AtPropagator when E/B fields are non-zero
   // -----------------------------------------------------------------------
   if (fEField.Mag2() != 0 || fBField.Mag2() != 0) {
      auto wrapModel = std::make_unique<ELossModelShared>(info.model);
      AtTools::AtPropagator prop(info.charge, info.mass, std::move(wrapModel));
      prop.SetEField(fEField);
      prop.SetBField(fBField);
      prop.SetState(iniPos, iniMom.Vect());

      AtTools::AtRK4AdaptiveStepper stepper;
      stepper.fInitialStep = fMaxPropStep;
      stepper.fMaxStep = fMaxPropStep;
      const double minAcceptedStepMm = stepper.fMinStep * 1e3 * kMinStepGuardScale;
      double length = 0;
      int numSteps = 0;
      int minStepSteps = 0;

      while (IsInVolume("drift_volume", prop.GetPosition())) {
         if (++numSteps > kMaxCurvedTransportSteps) {
            LOG(warning) << "Aborting curved SimpleSim track after " << numSteps
                         << " steps without leaving drift_volume";
            break;
         }

         double KE = AtTools::Kinematics::KE(prop.GetMomentum(), info.mass);
         if (KE <= fCurvedStopTol)
            break;

         auto mom4 = AtTools::Kinematics::Get4Vector(prop.GetMomentum(), info.mass);
         if (isnan(prop.GetPosition().X()) || isnan(prop.GetMomentum().X())) {
            LOG(error) << "Failed to simulate a point with nan!";
            return {{0, 0, 0}, {0, 0, 0, 0}};
         }
         if (!func(prop.GetPosition(), mom4))
            break;

         double KE_before = KE;
         prop.PropagateOneStep(stepper);

         auto &state = prop.GetState();
         if (state.status != AtTools::AtPropagator::StepStateStatus::kSuccess)
            break;

         double KE_after = AtTools::Kinematics::KE(prop.GetMomentum(), info.mass);
         double eLoss = KE_before - KE_after;
         if (eLoss < 0)
            eLoss = 0; // magnetic field does no work

         double stepDist = (prop.GetPosition() - state.fLastPos).R(); // mm
         if (stepDist <= minAcceptedStepMm || state.hUsed <= stepper.fMinStep * kMinStepGuardScale) {
            if (++minStepSteps > kMaxMinStepCurvedSteps) {
               LOG(warning) << "Aborting curved SimpleSim track after " << minStepSteps
                            << " minimum-size steps at position " << prop.GetPosition() << " with KE "
                            << KE_after << " MeV";
               break;
            }
         } else {
            minStepSteps = 0;
         }
         length += stepDist;

         auto newMom4 = AtTools::Kinematics::Get4Vector(prop.GetMomentum(), info.mass);
         AddHit(eLoss, prop.GetPosition(), newMom4, length);
      }

      return {prop.GetPosition(), AtTools::Kinematics::Get4Vector(prop.GetMomentum(), info.mass)};
   }

   // -----------------------------------------------------------------------
   // Straight-line fast path (zero field)
   // -----------------------------------------------------------------------
   auto &model = info.model;
   auto pos = iniPos;
   auto mom = iniMom;
   double length = 0;

   // Go until we exit the volume or the KE is less than 1keV
   while (IsInVolume("drift_volume", pos) && mom.E() - mom.M() > 1e-3 && func(pos, mom)) {

      if (isnan(pos.X()) || isnan(mom.X())) {
         LOG(error) << "Failed to simulate a point with nan!";
         return {{0, 0, 0}, {0, 0, 0, 0}};
      }
      // Direction particle is traveling
      auto dir = mom.Vect().Unit();

      // Get the energy loss from the model
      double KE = mom.E() - mom.M();
      double eLoss = model->GetEnergyLoss(KE, fDistStep);

      // Update the momentum from the energy loss model. Assume the energy loss does not change
      // the direction of the particle.
      auto E = mom.E() - eLoss;
      double p = sqrt(E * E - mom.M2());
      mom.SetPxPyPzE(dir.X() * p, dir.Y() * p, dir.Z() * p, E);

      LOG(debug) << mom << " " << mom.M() << " " << iniMom.M();

      pos += dir * fDistStep;
      length += fDistStep;
      AddHit(eLoss, pos, mom, length);
   }

   return {pos, mom};
}

std::pair<XYZPoint, PxPyPzEVector>
AtSimpleSimulation::TransportParticle(const ParticleInfo &info, int pdg, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                      const StepCallback &callback)
{
   fTrackID++;

   if (fEField.Mag2() != 0 || fBField.Mag2() != 0) {
      auto wrapModel = std::make_unique<ELossModelShared>(info.model);
      AtTools::AtPropagator prop(info.charge, info.mass, std::move(wrapModel));
      prop.SetEField(fEField);
      prop.SetBField(fBField);
      prop.SetState(iniPos, iniMom.Vect());

      AtTools::AtRK4AdaptiveStepper stepper;
      stepper.fInitialStep = fMaxPropStep;
      stepper.fMaxStep = fMaxPropStep;
      const double minAcceptedStepMm = stepper.fMinStep * 1e3 * kMinStepGuardScale;
      double length = 0;
      int numSteps = 0;
      int minStepSteps = 0;

      while (GetVolume(prop.GetPosition()) != nullptr) {
         if (++numSteps > kMaxCurvedTransportSteps) {
            LOG(warning) << "Aborting curved SimpleSim transport track after " << numSteps
                         << " steps without leaving the geometry";
            break;
         }

         double KE = AtTools::Kinematics::KE(prop.GetMomentum(), info.mass);
         if (KE <= fCurvedStopTol)
            break;

         auto momBefore = AtTools::Kinematics::Get4Vector(prop.GetMomentum(), info.mass);
         auto posBefore = prop.GetPosition();
         auto preVolumeName = GetVolumeName(posBefore);

         if (isnan(posBefore.X()) || isnan(prop.GetMomentum().X())) {
            LOG(error) << "Failed to transport a point with nan!";
            return {{0, 0, 0}, {0, 0, 0, 0}};
         }

         prop.PropagateOneStep(stepper);

         auto &state = prop.GetState();
         if (state.status != AtTools::AtPropagator::StepStateStatus::kSuccess)
            break;

         auto posAfter = prop.GetPosition();
         auto momAfter = AtTools::Kinematics::Get4Vector(prop.GetMomentum(), info.mass);
         double KE_after = AtTools::Kinematics::KE(prop.GetMomentum(), info.mass);
         double eLoss = KE - KE_after;
         if (eLoss < 0)
            eLoss = 0;

         double stepDist = (posAfter - state.fLastPos).R();
         if (stepDist <= minAcceptedStepMm || state.hUsed <= stepper.fMinStep * kMinStepGuardScale) {
            if (++minStepSteps > kMaxMinStepCurvedSteps) {
               LOG(warning) << "Aborting curved SimpleSim transport track after " << minStepSteps
                            << " minimum-size steps at position " << posAfter << " with KE " << KE_after
                            << " MeV for PDG " << pdg << " track " << fTrackID;
               break;
            }
         } else {
            minStepSteps = 0;
         }
         length += stepDist;

         if (callback) {
            TransportStep step;
            step.trackID = fTrackID;
            step.pdg = pdg;
            step.preVolumeName = preVolumeName;
            step.postVolumeName = GetVolumeName(posAfter);
            step.energyLoss = eLoss;
            step.length = length;
            step.trackMass = info.mass;
            step.prePosition = posBefore;
            step.postPosition = posAfter;
            step.preMomentum = momBefore;
            step.postMomentum = momAfter;
            if (!callback(step))
               break;
         }
      }

      return {prop.GetPosition(), AtTools::Kinematics::Get4Vector(prop.GetMomentum(), info.mass)};
   }

   auto &model = info.model;
   auto pos = iniPos;
   auto mom = iniMom;
   double length = 0;

   while (GetVolume(pos) != nullptr && mom.E() - mom.M() > 1e-3) {
      if (isnan(pos.X()) || isnan(mom.X())) {
         LOG(error) << "Failed to transport a point with nan!";
         return {{0, 0, 0}, {0, 0, 0, 0}};
      }

      auto posBefore = pos;
      auto momBefore = mom;
      auto preVolumeName = GetVolumeName(posBefore);
      auto dir = mom.Vect().Unit();
      double KE = mom.E() - mom.M();
      double eLoss = model->GetEnergyLoss(KE, fDistStep);
      auto E = mom.E() - eLoss;
      double p = sqrt(E * E - mom.M2());
      mom.SetPxPyPzE(dir.X() * p, dir.Y() * p, dir.Z() * p, E);
      pos += dir * fDistStep;
      length += fDistStep;

      if (callback) {
         TransportStep step;
         step.trackID = fTrackID;
         step.pdg = pdg;
         step.preVolumeName = preVolumeName;
         step.postVolumeName = GetVolumeName(pos);
         step.energyLoss = eLoss;
         step.length = length;
         step.trackMass = info.mass;
         step.prePosition = posBefore;
         step.postPosition = pos;
         step.preMomentum = momBefore;
         step.postMomentum = mom;
         if (!callback(step))
            break;
      }
   }

   return {pos, mom};
}

void AtSimpleSimulation::NewEvent()
{
   fMCPoints.Clear();
   fTrackID = 0;
}

/**
 * Units are mm, Mev, and Mev/c.
 */
void AtSimpleSimulation::AddHit(double ELoss, const XYZPoint &pos, const PxPyPzEVector &mom, double length)
{
   LOG(debug) << "Adding a hit at element " << fMCPoints.GetEntriesFast() << " in TClonesArray.";

   auto *mcPoint = dynamic_cast<AtMCPoint *>(fMCPoints.ConstructedAt(fMCPoints.GetEntriesFast(), "C"));

   mcPoint->SetTrackID(fTrackID);
   mcPoint->SetLength(length / 10.);      // Convert to cm
   mcPoint->SetEnergyLoss(ELoss / 1000.); // Convert to GeV
   mcPoint->SetVolName("drift_volume");

   if (fSCModel) {
      // In the simulation z = 0 is the window and z=1000 is the pad plane.
      // In the data analysis that is flipped, so we must adjust the z value, apply SC and move back
      auto posExpCoord = pos;
      posExpCoord.SetZ(1000 - pos.Z());
      auto corrExpCoord = fSCModel->ApplySpaceCharge(posExpCoord);
      corrExpCoord.SetZ(1000 + corrExpCoord.Z());
      mcPoint->SetPosition(corrExpCoord / 10.);
   } else
      mcPoint->SetPosition(pos / 10.);       // Convert to cm
   mcPoint->SetMomentum(mom.Vect() / 1000.); // Convert to GeV/c
   // mcPoint->Print(nullptr);
}

void AtSimpleSimulation::RegisterBranch(std::string branchName, bool perc)
{
   auto ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "The IO manager was not instatiated before attempting to simulate an event.";
      return;
   }

   ioMan->Register(branchName.c_str(), "AtTPC", &fMCPoints, perc);
}
