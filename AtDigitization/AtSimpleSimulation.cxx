
#include "AtSimpleSimulation.h"

#include "AtELossModel.h"
#include "AtELossModelFactory.h"
#include "AtKinematics.h"
#include "AtPropagator.h"

#include <FairLogger.h>

#include <TDatabasePDG.h>
#include <TGeoManager.h>
#include <TGeoMaterial.h>
#include <TGeoMedium.h>
#include <TGeoNavigator.h>
#include <TGeoNode.h>
#include <TGeoVolume.h>
#include <TParticlePDG.h>

#include <cmath>     // for sqrt
#include <stdexcept> // for invalid_argument
#include <utility>   // for pair

using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;
using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

namespace {
constexpr int kMaxMinStepCurvedSteps = 4096;
constexpr double kMinStepGuardScale = 1.01;

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

AtSimpleSimulation::AtSimpleSimulation(std::shared_ptr<AtTools::AtELossModelFactory> factory) : AtSimpleSimulation()
{
   fModelFactory = std::move(factory);
}

AtSimpleSimulation::AtSimpleSimulation(std::string geoFile, std::shared_ptr<AtTools::AtELossModelFactory> factory)
   : AtSimpleSimulation(std::move(geoFile))
{
   fModelFactory = std::move(factory);
}

bool AtSimpleSimulation::ParticleID::operator<(const ParticleID &other) const
{
   if (A < other.A) {
      return true;
   } else if (A > other.A) {
      return false;
   }
   return Z < other.Z;
}

TGeoVolume *AtSimpleSimulation::GetVolume(const XYZPoint &pos)
{
   auto pointCm = pos / 10.; // Convert from mm to cm

   std::lock_guard<std::mutex> lock(fGeoMutex);

   if (gGeoManager == nullptr) {
      return nullptr;
   }

   // Re-sync with gGeoManager if it changed (e.g. FairRunSim::Init loaded geometry).
   if (fGeoManager != gGeoManager || fNavigator == nullptr) {
      // The old navigator (if any) belongs to the old TGeoManager, which owns and
      // will delete it when the manager is destroyed. We abandon it intentionally.
      fNavigator = nullptr;
      fGeoManager = gGeoManager;
      fNavigator = fGeoManager->AddNavigator();
   }

   TGeoNode *node = fNavigator->FindNode(pointCm.X(), pointCm.Y(), pointCm.Z());
   if (node == nullptr) {
      return nullptr;
   }
   return node->GetVolume();
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
   static constexpr double kEperAMU = 931.494;            // MeV/c² per amu
   static constexpr double kEcharge = 1.602176634e-19;    // Coulombs

   ParticleID id = {.A = A, .Z = Z};
   fModels[id] = {model, Z * kEcharge, massAmu * kEperAMU};
}

std::pair<XYZPoint, PxPyPzEVector>
AtSimpleSimulation::TransportParticle(int Z, int A, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                      StepCallback callback)
{
   auto modelIt = fModels.find({A, Z});
   if (modelIt == fModels.end() && fModelFactory) {
      TryAutoCreateModel(Z, A, iniPos);
      modelIt = fModels.find({A, Z});
   }
   if (modelIt == fModels.end())
      throw std::invalid_argument("Missing energy loss model for Z:" + std::to_string(Z) + " A:" + std::to_string(A));
   if (GetVolume(iniPos) == nullptr)
      throw std::invalid_argument("Position of particle is outside the loaded geometry");

   return PropagateParticle(modelIt->second, GetPDGFromZA(Z, A), iniPos, iniMom, callback);
}

// ParticleInfo is taken by value (not const ref) so the transport loop owns its copy of the
// model shared_ptr and mass, independent of any external map modifications during transport.
std::pair<XYZPoint, PxPyPzEVector>
AtSimpleSimulation::PropagateParticle(ParticleInfo info, int pdg, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                      const StepCallback &callback)
{
   // -----------------------------------------------------------------------
   // Curved-track path: use AtPropagator when B field is non-zero or a
   // field function is provided. E-field alone does not trigger RK4 because
   // it is negligible for MeV-scale ion transport.
   // -----------------------------------------------------------------------
   if (fBField.Mag2() != 0 || fFieldFunc != nullptr) {
      AtTools::AtPropagator prop(info.charge, info.mass, info.model.get());

      // Initialize propagator fields
      if (fFieldFunc) {
         auto [eField, bField] = fFieldFunc(iniPos);
         prop.SetEField(eField);
         prop.SetBField(bField);
      } else {
         prop.SetEField(fEField);
         prop.SetBField(fBField);
      }
      prop.SetState(iniPos, iniMom.Vect());

      AtTools::AtRK4AdaptiveStepper stepper;
      stepper.fInitialStep = fMaxPropStep;
      stepper.fMaxStep = fMaxPropStep;
      const double minAcceptedStepMm = stepper.fMinStep * 1e3 * kMinStepGuardScale;
      double length = 0;
      int numSteps = 0;
      int minStepSteps = 0;

      TGeoVolume *curVol = nullptr;
      while ((curVol = GetVolume(prop.GetPosition())) != nullptr) {
         if (++numSteps > fMaxTransportSteps) {
            LOG(warning) << "Aborting curved SimpleSim track after " << numSteps
                         << " steps without leaving the geometry";
            break;
         }

         double KE = AtTools::Kinematics::KE(prop.GetMomentum(), info.mass);
         if (KE <= fStopTol)
            break;

         // For non-uniform fields, re-query the field at the current position before each RK4 step.
         // Within a single step, the field is treated as uniform (piecewise-constant approximation).
         if (fFieldFunc) {
            auto [eField, bField] = fFieldFunc(prop.GetPosition());
            prop.SetEField(eField);
            prop.SetBField(bField);
         }

         auto momBefore = AtTools::Kinematics::Get4Vector(prop.GetMomentum(), info.mass);
         auto posBefore = prop.GetPosition();
         std::string preVolumeName = curVol->GetName();

         if (std::isnan(posBefore.X()) || std::isnan(prop.GetMomentum().X())) {
            LOG(error) << "Failed to propagate a point with nan!";
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
            eLoss = 0; // magnetic field does no work

         double stepDist = (posAfter - state.fLastPos).R(); // mm
         if (stepDist <= minAcceptedStepMm || state.hUsed <= stepper.fMinStep * kMinStepGuardScale) {
            if (++minStepSteps > kMaxMinStepCurvedSteps) {
               LOG(warning) << "Aborting curved SimpleSim track after " << minStepSteps
                            << " minimum-size steps at position " << posAfter << " with KE " << KE_after
                            << " MeV for PDG " << pdg;
               break;
            }
         } else {
            minStepSteps = 0;
         }
         length += stepDist;

         if (callback) {
            TransportStep step;
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

   // -----------------------------------------------------------------------
   // Straight-line fast path (zero field)
   // KE and momentum are computed using info.mass throughout (not the 4-vector's invariant
   // mass) to stay consistent with the curved path and the energy loss model's mass.
   // -----------------------------------------------------------------------
   auto &model = info.model;
   auto pos = iniPos;
   auto mom = iniMom;
   double length = 0;
   int numSteps = 0;

   TGeoVolume *curVol = nullptr;
   while ((curVol = GetVolume(pos)) != nullptr) {
      if (++numSteps > fMaxTransportSteps) {
         LOG(warning) << "Aborting straight-line SimpleSim track after " << numSteps
                      << " steps without leaving the geometry";
         break;
      }

      double KE = AtTools::Kinematics::KE(mom.Vect(), info.mass);
      if (KE <= fStopTol)
         break;

      if (std::isnan(pos.X()) || std::isnan(mom.X())) {
         LOG(error) << "Failed to propagate a point with nan!";
         return {{0, 0, 0}, {0, 0, 0, 0}};
      }

      auto posBefore = pos;
      auto momBefore = mom;
      std::string preVolumeName = curVol->GetName();
      auto dir = mom.Vect().Unit();
      double eLoss = model->GetEnergyLoss(KE, fDistStep);
      double newKE = KE - eLoss;
      if (newKE <= 0)
         break;
      double E = newKE + info.mass;
      double p = sqrt(E * E - info.mass * info.mass);
      mom.SetPxPyPzE(dir.X() * p, dir.Y() * p, dir.Z() * p, E);
      pos += dir * fDistStep;
      length += fDistStep;

      if (callback) {
         TransportStep step;
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

void AtSimpleSimulation::TryAutoCreateModel(int Z, int A, const XYZPoint &pos)
{
   if (!fModelFactory)
      return;

   TGeoVolume *volume = GetVolume(pos);
   if (volume == nullptr) {
      LOG(warning) << "TryAutoCreateModel: position " << pos << " is outside geometry; cannot determine material";
      return;
   }

   TGeoMedium *medium = volume->GetMedium();
   if (medium == nullptr) {
      LOG(warning) << "TryAutoCreateModel: volume " << volume->GetName() << " has no medium";
      return;
   }

   TGeoMaterial *material = medium->GetMaterial();
   if (material == nullptr) {
      LOG(warning) << "TryAutoCreateModel: medium " << medium->GetName() << " has no material";
      return;
   }

   // Look up mass in amu from PDG database for precision; fall back to A
   double massAmu = static_cast<double>(A);
   int pdgCode = GetPDGFromZA(Z, A);
   TParticlePDG *particle = TDatabasePDG::Instance()->GetParticle(pdgCode);
   if (particle != nullptr)
      massAmu = particle->Mass() / 0.931494; // GeV/c² -> amu

   auto model = fModelFactory->CreateModel(Z, A, massAmu, material);
   if (model) {
      AddModel(Z, A, model, massAmu);
      LOG(info) << "Auto-created energy loss model for Z=" << Z << " A=" << A << " in " << material->GetName();
   } else {
      LOG(warning) << "Factory failed to create energy loss model for Z=" << Z << " A=" << A << " in "
                   << material->GetName();
   }
}
