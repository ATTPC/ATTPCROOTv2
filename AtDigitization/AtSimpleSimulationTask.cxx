#include "AtSimpleSimulationTask.h"

#include "AtDetectorList.h"
#include "AtMCTrack.h"
#include "AtSimpleSimulation.h"
#include "AtTpc/AtTpc.h"

#include <FairField.h>
#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h>
#include <Math/Vector3D.h>
#include <Math/Vector4D.h>
#include <Math/Vector4Dfwd.h>
#include <TClonesArray.h>
#include <TDatabasePDG.h>
#include <TGeoBBox.h>
#include <TGeoManager.h>
#include <TGeoVolume.h>
#include <TParticlePDG.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

using namespace ROOT::Math;

namespace {
// SimpleSim uses mm/MeV; FairRoot/AtTpc uses cm/GeV.
constexpr double kCmToMm = 10.;
constexpr double kMmToCm = 0.1;
constexpr double kGeVToMeV = 1000.;
constexpr double kMeVToGeV = 0.001;

std::pair<int, int> GetZAFromPDG(int pdg)
{
   if (pdg > 1000000000) {
      int A = (pdg / 10) % 1000;
      int Z = (pdg / 10000) % 1000;
      return {Z, A};
   }

   TParticlePDG *particle = TDatabasePDG::Instance()->GetParticle(pdg);
   if (particle != nullptr) {
      int Z = static_cast<int>(std::round(particle->Charge() / 3.0));
      int A = static_cast<int>(std::round(particle->Mass() / 0.9315));
      return {Z, std::max(A, 1)};
   }

   return {0, 0};
}
} // namespace

AtSimpleSimulationTask::AtSimpleSimulationTask(std::unique_ptr<AtSimpleSimulation> sim) : fSimulation(std::move(sim)) {}

InitStatus AtSimpleSimulationTask::Init()
{
   if (fDetector == nullptr) {
      LOG(fatal) << "AtSimpleSimulationTask requires a sensitive detector. "
                 << "Call SetDetector(tpc) before Init(). "
                 << "For standalone simulation, use AtSimpleSimulation::SimulateParticle() directly.";
      return kFATAL;
   }
   LOG(info) << "AtSimpleSimulationTask: using detector-coupled transport adapter";
   fDetector->SetStopOnReactionVolumeExit(true);

   if (fAutoConfigureField)
      ConfigureFieldFromFairRun();

   auto sourceStatus = InitEventSource();
   if (sourceStatus != kSUCCESS)
      return sourceStatus;

   RegisterMCTrackBranch();
   return kSUCCESS;
}

void AtSimpleSimulationTask::Exec(Option_t *)
{
   fSimulation->NewEvent();

   auto eventState = LoadEvent();
   if (!eventState.hasEvent)
      return;

   FillMCTracks();
   if (!eventState.transportPrimaries)
      return;

   TransportCurrentEvent(eventState.beamEvent);
}

void AtSimpleSimulationTask::Finish() { FinishEventSource(); }

InitStatus AtSimpleSimulationTask::InitEventSource() { return kSUCCESS; }

void AtSimpleSimulationTask::FinishEventSource() {}

void AtSimpleSimulationTask::ConfigureFieldFromFairRun()
{
   using XYZVector = ROOT::Math::XYZVector;
   constexpr double kKGtoTesla = 0.1;

   // Skip if the simulation already has a manually configured non-zero field
   // (check by seeing if B is non-zero -- user set it before Init)
   // We can't access the private fBField directly, so we rely on the convention
   // that auto-config runs first and manual overrides come before Init().

   auto *run = FairRun::Instance();
   if (run == nullptr) {
      LOG(info) << "AtSimpleSimulationTask: no FairRun instance; skipping field auto-config";
      return;
   }

   auto *field = run->GetField();
   if (field == nullptr) {
      LOG(info) << "AtSimpleSimulationTask: no field set on FairRun; SimpleSim fields remain at zero";
      return;
   }

   // Find drift volume center for field sampling
   double cx = 0, cy = 0, cz = 0;
   TGeoVolume *driftVol = nullptr;
   if (gGeoManager != nullptr)
      driftVol = gGeoManager->FindVolumeFast("drift_volume");

   if (driftVol != nullptr) {
      auto *shape = dynamic_cast<TGeoBBox *>(driftVol->GetShape());
      if (shape != nullptr) {
         const double *origin = shape->GetOrigin();
         cx = origin[0];
         cy = origin[1];
         cz = origin[2];
      }
   }

   // Sample field at drift volume center (FairField returns kG, position in cm)
   double bx_kG = field->GetBx(cx, cy, cz);
   double by_kG = field->GetBy(cx, cy, cz);
   double bz_kG = field->GetBz(cx, cy, cz);

   double bx_T = bx_kG * kKGtoTesla;
   double by_T = by_kG * kKGtoTesla;
   double bz_T = bz_kG * kKGtoTesla;

   fSimulation->SetMagneticField(XYZVector(bx_T, by_T, bz_T));
   LOG(info) << "AtSimpleSimulationTask: auto-configured B field from FairRun: (" << bx_T << ", " << by_T << ", " << bz_T
             << ") T (sampled at drift volume center)";

   // Warn if field is not constant (SimpleSim assumes uniform)
   if (field->GetType() != 0)
      LOG(warning) << "AtSimpleSimulationTask: SimpleSim assumes uniform fields, but the FairRun field type is "
                   << field->GetType() << " (non-constant). Using field value sampled at drift volume center.";

   // For constant fields, check if drift volume extends beyond field region
   if (field->GetType() == 0 && driftVol != nullptr) {
      auto *shape = dynamic_cast<TGeoBBox *>(driftVol->GetShape());
      if (shape != nullptr) {
         const double *origin = shape->GetOrigin();
         double dx = shape->GetDX();
         double dy = shape->GetDY();
         double dz = shape->GetDZ();

         // Check corners of drift volume bounding box
         double corners[8][3] = {{origin[0] - dx, origin[1] - dy, origin[2] - dz},
                                 {origin[0] + dx, origin[1] - dy, origin[2] - dz},
                                 {origin[0] - dx, origin[1] + dy, origin[2] - dz},
                                 {origin[0] + dx, origin[1] + dy, origin[2] - dz},
                                 {origin[0] - dx, origin[1] - dy, origin[2] + dz},
                                 {origin[0] + dx, origin[1] - dy, origin[2] + dz},
                                 {origin[0] - dx, origin[1] + dy, origin[2] + dz},
                                 {origin[0] + dx, origin[1] + dy, origin[2] + dz}};

         for (const auto &corner : corners) {
            double bz_corner = field->GetBz(corner[0], corner[1], corner[2]);
            if (std::abs(bz_corner - bz_kG) > 1e-6) {
               LOG(warning) << "AtSimpleSimulationTask: drift volume extends beyond the constant field region. "
                            << "Field at corner (" << corner[0] << ", " << corner[1] << ", " << corner[2]
                            << ") cm differs from center value.";
               break;
            }
         }
      }
   }
}

void AtSimpleSimulationTask::RegisterMCTrackBranch()
{
   auto *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "The IO manager was not instantiated before AtSimpleSimulationTask::Init().";
      return;
   }

   auto *existing = dynamic_cast<TClonesArray *>(ioMan->GetObject("MCTrack"));
   if (existing != nullptr) {
      fMCTrackArray = existing;
      return;
   }

   if (fMCTrackArray == nullptr)
      fMCTrackArray = new TClonesArray("AtMCTrack");

   ioMan->Register("MCTrack", "Stack", fMCTrackArray, kTRUE);
}

void AtSimpleSimulationTask::FillMCTracks()
{
   if (fMCTrackArray == nullptr)
      return;

   fMCTrackArray->Clear("C");
   for (const auto &particle : fCollector.GetParticles()) {
      new ((*fMCTrackArray)[particle.trackID]) AtMCTrack(particle.pdgCode, -1, particle.px, particle.py, particle.pz,
                                                         particle.vx, particle.vy, particle.vz, 0.0, 0);
   }
}

void AtSimpleSimulationTask::TransportCurrentEvent(bool beamEvent)
{
   for (const auto &particle : fCollector.GetParticles())
      TransportParticle(particle, beamEvent);
}

void AtSimpleSimulationTask::TransportParticle(const AtCollectedParticle &particle, bool beamEvent)
{
   auto [Z, A] = GetZAFromPDG(particle.pdgCode);
   if (Z == 0 && A == 0)
      return;

   XYZPoint pos(particle.vx * kCmToMm, particle.vy * kCmToMm, particle.vz * kCmToMm);
   PxPyPzEVector mom(particle.px * kGeVToMeV, particle.py * kGeVToMeV, particle.pz * kGeVToMeV,
                     particle.e * kGeVToMeV);

   try {
      if (!IsSensitiveVolume(fSimulation->GetVolumeNameAt(pos)))
         pos = FindSensitiveEntry(pos, mom);

      LOG(info) << "Simulating particle Z=" << Z << " A=" << A << " with initial pos=" << pos << " mm and mom=" << mom
                << " MeV/c";

      const bool beamTrack = beamEvent && particle.trackID == 0;
      if (IsSensitiveVolume(fSimulation->GetVolumeNameAt(pos)) &&
          !SubmitInitialSensitivePoint(particle.trackID, particle.pdgCode, beamTrack, pos, mom))
         return;

      fSimulation->TransportParticle(
         Z, A, pos, mom, [this, trackID = particle.trackID, beamEvent](const AtSimpleSimulation::TransportStep &step) {
            const bool preSensitive = IsSensitiveVolume(step.preVolumeName);
            const bool postSensitive = IsSensitiveVolume(step.postVolumeName);
            const bool entering = !preSensitive && postSensitive;
            const bool exiting = preSensitive && !postSensitive;
            const bool currentBeamTrack = beamEvent && trackID == 0;
            const bool keepTransporting =
               ProcessDetectorStep(step, trackID, currentBeamTrack, preSensitive, postSensitive, entering, exiting);
            if (exiting && !postSensitive)
               return false;
            return keepTransporting;
         });
   } catch (const std::invalid_argument &ex) {
      LOG(fatal) << "AtSimpleSimulationTask: skipping particle Z=" << Z << " A=" << A << ": " << ex.what();
   }
}

bool AtSimpleSimulationTask::SubmitInitialSensitivePoint(int trackID, int pdg, bool beamTrack, const XYZPoint &pos,
                                                         const PxPyPzEVector &mom)
{
   AtTpc::StepState detectorStep;
   detectorStep.trackID = trackID;
   detectorStep.pdg = pdg;
   detectorStep.volumeName = fSimulation->GetVolumeNameAt(pos).c_str();
   detectorStep.volumeID = kAtTpc;
   detectorStep.detCopyID = 0;
   detectorStep.beamTrack = beamTrack;
   detectorStep.entering = true;
   detectorStep.exiting = false;
   detectorStep.stopping = (mom.E() - mom.M() <= 1e-3);
   detectorStep.disappeared = false;
   detectorStep.energyLoss = 0.0;
   detectorStep.timeNs = 0.0;
   detectorStep.trackLength = 0.0;
   detectorStep.totalEnergy = mom.E() * kMeVToGeV;
   detectorStep.trackMass = mom.M() * kMeVToGeV;
   detectorStep.pos.SetXYZT(pos.X() * kMmToCm, pos.Y() * kMmToCm, pos.Z() * kMmToCm, 0.0);
   detectorStep.mom.SetXYZT(mom.Px() * kMeVToGeV, mom.Py() * kMeVToGeV, mom.Pz() * kMeVToGeV, mom.E() * kMeVToGeV);
   detectorStep.posOut = detectorStep.pos;
   detectorStep.momOut = detectorStep.mom;

   return !fDetector->ProcessStep(detectorStep);
}

bool AtSimpleSimulationTask::ProcessDetectorStep(const AtSimpleSimulation::TransportStep &step, int trackID, bool beamTrack,
                                                 bool preSensitive, bool postSensitive, bool entering, bool exiting)
{
   if (!preSensitive && !postSensitive)
      return true;

   AtTpc::StepState detectorStep;
   detectorStep.trackID = trackID;
   detectorStep.pdg = step.pdg;
   detectorStep.volumeName = postSensitive ? step.postVolumeName.c_str() : step.preVolumeName.c_str();
   detectorStep.volumeID = kAtTpc;
   detectorStep.detCopyID = 0;
   detectorStep.beamTrack = beamTrack;
   detectorStep.entering = entering;
   detectorStep.exiting = exiting;
   detectorStep.stopping = postSensitive && (step.postMomentum.E() - step.postMomentum.M() <= 1e-3);
   detectorStep.disappeared = false;
   detectorStep.energyLoss = step.energyLoss * kMeVToGeV;
   detectorStep.timeNs = 0.;
   detectorStep.trackLength = step.length * kMmToCm;

   const auto &refPos = postSensitive ? step.postPosition : step.prePosition;
   const auto &refMom = postSensitive ? step.postMomentum : step.preMomentum;
   detectorStep.totalEnergy = refMom.E() * kMeVToGeV;
   detectorStep.trackMass = step.trackMass * kMeVToGeV;
   detectorStep.pos.SetXYZT(refPos.X() * kMmToCm, refPos.Y() * kMmToCm, refPos.Z() * kMmToCm, 0.);
   detectorStep.mom.SetXYZT(refMom.Px() * kMeVToGeV, refMom.Py() * kMeVToGeV, refMom.Pz() * kMeVToGeV,
                            refMom.E() * kMeVToGeV);
   detectorStep.posOut.SetXYZT(step.postPosition.X() * kMmToCm, step.postPosition.Y() * kMmToCm,
                               step.postPosition.Z() * kMmToCm, 0.);
   detectorStep.momOut.SetXYZT(step.postMomentum.Px() * kMeVToGeV, step.postMomentum.Py() * kMeVToGeV,
                               step.postMomentum.Pz() * kMeVToGeV, step.postMomentum.E() * kMeVToGeV);

   const bool stopTransport = fDetector->ProcessStep(detectorStep);
   return !stopTransport;
}

XYZPoint AtSimpleSimulationTask::FindSensitiveEntry(const XYZPoint &pos, const PxPyPzEVector &mom) const
{
   const auto dir = mom.Vect().Unit();
   if (dir.R() == 0.0)
      throw std::invalid_argument("Particle momentum is zero; cannot search for detector entry");

   constexpr double stepMm = 1.0;
   constexpr int maxSteps = 5000;
   auto probe = pos;
   for (int i = 0; i < maxSteps; ++i) {
      probe += dir * stepMm;
      if (IsSensitiveVolume(fSimulation->GetVolumeNameAt(probe)))
         return probe;
   }

   throw std::invalid_argument("Particle does not intersect a sensitive detector volume");
}

bool AtSimpleSimulationTask::IsSensitiveVolume(const std::string &volumeName)
{
   return AtTpc::IsSensitiveVolume(volumeName);
}

ClassImp(AtSimpleSimulationTask);
