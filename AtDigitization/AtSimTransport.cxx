#include "AtSimTransport.h"

#include "AtELossManager.h"
#include "AtELossModel.h"
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

#include <cmath>
#include <stdexcept>
#include <utility>

using ModelPtr = std::shared_ptr<AtTools::AtELossModel>;
using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;
using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

namespace {
constexpr double kAmuToMeV = 931.494;        // MeV/c² per amu
constexpr double kEcharge = 1.602176634e-19; // Coulombs

int GetPDGFromZA(int Z, int A)
{
   if (A == 1 && Z == 1)
      return 2212;
   if (A == 1 && Z == 0)
      return 2112;
   return 1000000000 + Z * 10000 + A * 10;
}

double GetMassAmuFromPDG(int Z, int A)
{
   int pdg = GetPDGFromZA(Z, A);
   TParticlePDG *particle = TDatabasePDG::Instance()->GetParticle(pdg);
   if (particle != nullptr) {
      return particle->Mass() / 0.931494; // GeV/c² -> amu
   }
   return static_cast<double>(A);
}
} // namespace

AtSimTransport::AtSimTransport() : fManager(std::make_shared<AtTools::AtELossManager>())
{
   // Defer geometry check until first use; gGeoManager may not be populated yet.
}

AtSimTransport::AtSimTransport(std::shared_ptr<AtTools::AtELossManager> manager) : fManager(std::move(manager))
{
   if (!fManager)
      fManager = std::make_shared<AtTools::AtELossManager>();
}

AtSimTransport::AtSimTransport(std::string geoFile)
   : AtSimTransport(std::move(geoFile), std::make_shared<AtTools::AtELossManager>())
{
}

AtSimTransport::AtSimTransport(std::string geoFile, std::shared_ptr<AtTools::AtELossManager> manager)
   : fManager(std::move(manager))
{
   if (!fManager)
      fManager = std::make_shared<AtTools::AtELossManager>();
   TGeoManager *geo = TGeoManager::Import(geoFile.c_str());
   if (gGeoManager == nullptr)
      LOG(fatal) << "Failed to load geometry file " << geoFile << " " << geo;
}

void AtSimTransport::AddModel(int Z, int A, ModelPtr model)
{
   if (!fManager)
      LOG(fatal) << "AtSimTransport::AddModel: no AtELossManager attached";
   fManager->AddModel(Z, A, std::move(model));
}

void AtSimTransport::AddModel(int Z, int A, const std::string &materialName, ModelPtr model)
{
   if (!fManager)
      LOG(fatal) << "AtSimTransport::AddModel: no AtELossManager attached";
   fManager->AddModel(Z, A, materialName, std::move(model));
}

TGeoVolume *AtSimTransport::GetVolume(const XYZPoint &pos)
{
   auto pointCm = pos / 10.; // mm → cm (TGeo)

   std::lock_guard<std::mutex> lock(fGeoMutex);

   if (gGeoManager == nullptr)
      return nullptr;

   // Re-sync with gGeoManager if it changed (e.g. FairRunSim::Init loaded geometry).
   // A private navigator is required because AtMCFitter runs many simulations that share
   // gGeoManager's state, and its default navigator is not safe to share.
   if (fGeoManager != gGeoManager || fNavigator == nullptr) {
      fNavigator = nullptr; // old navigator is owned by the old manager; abandon it.
      fGeoManager = gGeoManager;
      fNavigator = fGeoManager->AddNavigator();
   }

   TGeoNode *node = fNavigator->FindNode(pointCm.X(), pointCm.Y(), pointCm.Z());
   if (node == nullptr)
      return nullptr;
   return node->GetVolume();
}

std::string AtSimTransport::GetVolumeName(const XYZPoint &point)
{
   TGeoVolume *volume = GetVolume(point);
   return volume != nullptr ? volume->GetName() : std::string{};
}

TGeoMaterial *AtSimTransport::GetMaterial(const XYZPoint &pos)
{
   TGeoVolume *vol = GetVolume(pos);
   if (vol == nullptr)
      return nullptr;
   TGeoMedium *medium = vol->GetMedium();
   return medium != nullptr ? medium->GetMaterial() : nullptr;
}

AtSimTransport::ParticleInfo AtSimTransport::LookupParticleInfo(int Z, int A, const XYZPoint &pos)
{
   ParticleInfo info;
   if (!fManager)
      return info;

   TGeoMaterial *material = GetMaterial(pos);
   double massAmu = GetMassAmuFromPDG(Z, A);
   info.model = fManager->GetModel(Z, A, massAmu, material);
   info.charge = Z * kEcharge;
   info.mass = massAmu * kAmuToMeV;
   return info;
}

std::pair<XYZPoint, PxPyPzEVector> AtSimTransport::TransportParticle(int Z, int A, const XYZPoint &iniPos,
                                                                         const PxPyPzEVector &iniMom,
                                                                         StepCallback callback)
{
   if (GetVolume(iniPos) == nullptr)
      throw std::invalid_argument("Position of particle is outside the loaded geometry");

   ParticleInfo info = LookupParticleInfo(Z, A, iniPos);
   if (!info.model) {
      throw std::invalid_argument("No energy-loss model available for Z=" + std::to_string(Z) +
                                  " A=" + std::to_string(A) + " in material at start position");
   }

   return PropagateParticle(Z, A, GetPDGFromZA(Z, A), iniPos, iniMom, callback);
}

std::pair<XYZPoint, PxPyPzEVector> AtSimTransport::PropagateParticle(int Z, int A, int pdg, const XYZPoint &iniPos,
                                                                         const PxPyPzEVector &iniMom,
                                                                         const StepCallback &callback)
{
   if (fBField.Mag2() != 0 || fFieldFunc != nullptr)
      return PropagateCurved(Z, A, pdg, iniPos, iniMom, callback);
   return PropagateStraightLine(Z, A, pdg, iniPos, iniMom, callback);
}

AtSimTransport::TransportStep
AtSimTransport::BuildStep(int pdg, std::string preVolumeName, std::string postVolumeName, std::string materialName,
                              const XYZPoint &posBefore, const XYZPoint &posAfter, const PxPyPzEVector &momBefore,
                              const PxPyPzEVector &momAfter, double eLoss, double length, double mass)
{
   TransportStep step;
   step.pdg = pdg;
   step.preVolumeName = std::move(preVolumeName);
   step.postVolumeName = std::move(postVolumeName);
   step.materialName = std::move(materialName);
   step.energyLoss = eLoss;
   step.length = length;
   step.trackMass = mass;
   step.prePosition = posBefore;
   step.postPosition = posAfter;
   step.preMomentum = momBefore;
   step.postMomentum = momAfter;
   return step;
}

std::pair<XYZPoint, PxPyPzEVector> AtSimTransport::PropagateCurved(int Z, int A, int pdg, const XYZPoint &iniPos,
                                                                       const PxPyPzEVector &iniMom,
                                                                       const StepCallback &callback)
{
   ParticleInfo info = LookupParticleInfo(Z, A, iniPos);
   if (!info.model) {
      LOG(error) << "PropagateCurved: no model for Z=" << Z << " A=" << A << " at start position";
      return {iniPos, iniMom};
   }

   AtTools::AtPropagator prop(info.charge, info.mass, info.model.get());

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
   stepper.fInitialStep = fMaxStep;
   stepper.fMaxStep = fMaxStep;
   const double minAcceptedStepMm = stepper.fMinStep * 1e3 * fMinStepGuardScale;

   double length = 0;
   int numSteps = 0;
   int minStepSteps = 0;

   TGeoVolume *curVol = nullptr;
   std::string curMaterialName;
   while ((curVol = GetVolume(prop.GetPosition())) != nullptr) {
      if (++numSteps > fMaxTransportSteps) {
         LOG(warning) << "Aborting curved SimpleSim track after " << numSteps << " steps without leaving the geometry";
         break;
      }

      // Re-query the manager on volume change so each medium gets its own model.
      TGeoMedium *medium = curVol->GetMedium();
      TGeoMaterial *material = medium != nullptr ? medium->GetMaterial() : nullptr;
      const std::string newMaterialName = material != nullptr ? material->GetName() : std::string{};
      if (newMaterialName != curMaterialName) {
         auto newModel = fManager ? fManager->GetModel(Z, A, info.mass / kAmuToMeV, material) : ModelPtr{};
         if (!newModel) {
            LOG(warning) << "PropagateCurved: no model for material '" << newMaterialName << "'; stopping track";
            break;
         }
         info.model = newModel;
         prop.SetELossModel(info.model.get());
         curMaterialName = newMaterialName;
      }

      double KE = AtTools::Kinematics::KE(prop.GetMomentum(), info.mass);
      if (KE <= fStopTol)
         break;

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
      if (stepDist <= minAcceptedStepMm || state.hUsed <= stepper.fMinStep * fMinStepGuardScale) {
         if (++minStepSteps > fMaxMinStepStreak) {
            LOG(warning) << "Aborting curved SimpleSim track after " << minStepSteps
                         << " minimum-size steps at position " << posAfter << " with KE " << KE_after << " MeV for PDG "
                         << pdg;
            break;
         }
      } else {
         minStepSteps = 0;
      }
      length += stepDist;

      if (callback) {
         auto step = BuildStep(pdg, preVolumeName, GetVolumeName(posAfter), curMaterialName, posBefore, posAfter,
                               momBefore, momAfter, eLoss, length, info.mass);
         if (!callback(step))
            break;
      }
   }

   return {prop.GetPosition(), AtTools::Kinematics::Get4Vector(prop.GetMomentum(), info.mass)};
}

std::pair<XYZPoint, PxPyPzEVector>
AtSimTransport::PropagateStraightLine(int Z, int A, int pdg, const XYZPoint &iniPos, const PxPyPzEVector &iniMom,
                                          const StepCallback &callback)
{
   ParticleInfo info = LookupParticleInfo(Z, A, iniPos);
   if (!info.model) {
      LOG(error) << "PropagateStraightLine: no model for Z=" << Z << " A=" << A << " at start position";
      return {iniPos, iniMom};
   }

   auto pos = iniPos;
   auto mom = iniMom;
   double length = 0;
   int numSteps = 0;

   TGeoVolume *curVol = nullptr;
   std::string curMaterialName;
   while ((curVol = GetVolume(pos)) != nullptr) {
      if (++numSteps > fMaxTransportSteps) {
         LOG(warning) << "Aborting straight-line SimpleSim track after " << numSteps
                      << " steps without leaving the geometry";
         break;
      }

      // Re-query the manager on volume change.
      TGeoMedium *medium = curVol->GetMedium();
      TGeoMaterial *material = medium != nullptr ? medium->GetMaterial() : nullptr;
      const std::string newMaterialName = material != nullptr ? material->GetName() : std::string{};
      if (newMaterialName != curMaterialName) {
         auto newModel = fManager ? fManager->GetModel(Z, A, info.mass / kAmuToMeV, material) : ModelPtr{};
         if (!newModel) {
            LOG(warning) << "PropagateStraightLine: no model for material '" << newMaterialName << "'; stopping track";
            break;
         }
         info.model = newModel;
         curMaterialName = newMaterialName;
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
      double eLoss = info.model->GetEnergyLoss(KE, fMaxStep);
      double newKE = KE - eLoss;
      if (newKE <= 0)
         break;
      double E = newKE + info.mass;
      double p = std::sqrt(E * E - info.mass * info.mass);
      mom.SetPxPyPzE(dir.X() * p, dir.Y() * p, dir.Z() * p, E);
      pos += dir * fMaxStep;
      length += fMaxStep;

      if (callback) {
         auto step = BuildStep(pdg, preVolumeName, GetVolumeName(pos), curMaterialName, posBefore, pos, momBefore, mom,
                               eLoss, length, info.mass);
         if (!callback(step))
            break;
      }
   }

   return {pos, mom};
}
