#include "AtSimTransportTask.h"

#include "AtDetectorList.h"
#include "AtMCTrack.h"
#include "AtSimTransport.h"

#include <FairField.h>
#include <FairLogger.h>
#include <FairRootManager.h>
#include <FairRun.h>
#include <FairRunSim.h>

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
#include <TObjArray.h>
#include <TParticle.h>
#include <TParticlePDG.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

#include "AtTpc/AtTpc.h"

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

AtSimTransportTask::AtSimTransportTask(std::unique_ptr<AtSimTransport> sim) : fSimulation(std::move(sim)) {}

InitStatus AtSimTransportTask::Init()
{
   // Auto-discover detector from FairRunSim if not set manually
   if (fDetector == nullptr) {
      auto *runSim = FairRunSim::Instance();
      if (runSim != nullptr) {
         auto *modules = runSim->GetListOfModules();
         if (modules != nullptr) {
            for (int i = 0; i < modules->GetEntries(); ++i) {
               auto *det = dynamic_cast<AtTpc *>(modules->At(i));
               if (det != nullptr) {
                  fDetector = det;
                  LOG(info) << "AtSimTransportTask: auto-discovered AtTpc detector '" << det->GetName()
                            << "' from FairRunSim";
                  break;
               }
            }
         }
      }
   }
   if (fDetector == nullptr) {
      LOG(fatal) << "AtSimTransportTask requires a sensitive detector. "
                 << "Call SetDetector(tpc) before Init(), or register an AtTpc with FairRunSim.";
      return kFATAL;
   }
   LOG(info) << "AtSimTransportTask: using detector-coupled transport adapter";

   if (fAutoConfigureField)
      ConfigureFieldFromFairRun();

   auto sourceStatus = InitEventSource();
   if (sourceStatus != kSUCCESS)
      return sourceStatus;

   RegisterMCTrackBranch();
   return kSUCCESS;
}

void AtSimTransportTask::Exec(Option_t *)
{
   auto eventState = LoadEvent();
   if (!eventState.hasEvent)
      return;

   FillMCTracks();
   if (!eventState.transportPrimaries)
      return;

   TransportCurrentEvent(eventState.beamEvent);
}

void AtSimTransportTask::Finish()
{
   FinishEventSource();
}

InitStatus AtSimTransportTask::InitEventSource()
{
   return kSUCCESS;
}

void AtSimTransportTask::FinishEventSource() {}

void AtSimTransportTask::ConfigureFieldFromFairRun()
{
   using XYZVector = ROOT::Math::XYZVector;
   constexpr double kKGtoTesla = 0.1;

   // Skip if the simulation already has a manually configured non-zero field
   // (check by seeing if B is non-zero -- user set it before Init)
   // We can't access the private fBField directly, so we rely on the convention
   // that auto-config runs first and manual overrides come before Init().

   auto *run = FairRun::Instance();
   if (run == nullptr) {
      LOG(info) << "AtSimTransportTask: no FairRun instance; skipping field auto-config";
      return;
   }

   auto *field = run->GetField();
   if (field == nullptr) {
      LOG(info) << "AtSimTransportTask: no field set on FairRun; SimpleSim fields remain at zero";
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
   LOG(info) << "AtSimTransportTask: auto-configured B field from FairRun: (" << bx_T << ", " << by_T << ", "
             << bz_T << ") T (sampled at drift volume center)";

   // FairField::GetType() == 0 means constant field. For non-constant fields (maps, etc.),
   // set up a per-step field query so the propagator sees the correct field at each position.
   // The lambda captures the FairField pointer (owned by FairRun, outlives the simulation).
   if (field->GetType() != 0) {
      LOG(info) << "AtSimTransportTask: non-constant field (type " << field->GetType()
                << "); enabling per-step field queries";
      fSimulation->SetFieldFunction(
         [field](const ROOT::Math::XYZPoint &pos_mm) -> std::pair<ROOT::Math::XYZVector, ROOT::Math::XYZVector> {
            constexpr double kMmToCm = 0.1;
            constexpr double kKGtoT = 0.1;
            double x_cm = pos_mm.X() * kMmToCm;
            double y_cm = pos_mm.Y() * kMmToCm;
            double z_cm = pos_mm.Z() * kMmToCm;
            ROOT::Math::XYZVector B(field->GetBx(x_cm, y_cm, z_cm) * kKGtoT, field->GetBy(x_cm, y_cm, z_cm) * kKGtoT,
                                    field->GetBz(x_cm, y_cm, z_cm) * kKGtoT);
            return {ROOT::Math::XYZVector(0, 0, 0), B};
         });
   }

   // For constant fields, check if drift volume extends beyond field region
   if (field->GetType() == 0 && driftVol != nullptr) {
      auto *shape = dynamic_cast<TGeoBBox *>(driftVol->GetShape());
      if (shape != nullptr) {
         const double *origin = shape->GetOrigin();
         double dx = shape->GetDX();
         double dy = shape->GetDY();
         double dz = shape->GetDZ();

         // Check corners of drift volume bounding box
         double corners[8][3] = {
            {origin[0] - dx, origin[1] - dy, origin[2] - dz}, {origin[0] + dx, origin[1] - dy, origin[2] - dz},
            {origin[0] - dx, origin[1] + dy, origin[2] - dz}, {origin[0] + dx, origin[1] + dy, origin[2] - dz},
            {origin[0] - dx, origin[1] - dy, origin[2] + dz}, {origin[0] + dx, origin[1] - dy, origin[2] + dz},
            {origin[0] - dx, origin[1] + dy, origin[2] + dz}, {origin[0] + dx, origin[1] + dy, origin[2] + dz}};

         for (const auto &corner : corners) {
            double bz_corner = field->GetBz(corner[0], corner[1], corner[2]);
            if (std::abs(bz_corner - bz_kG) > 1e-6) {
               LOG(warning) << "AtSimTransportTask: drift volume extends beyond the constant field region. "
                            << "Field at corner (" << corner[0] << ", " << corner[1] << ", " << corner[2]
                            << ") cm differs from center value.";
               break;
            }
         }
      }
   }
}

void AtSimTransportTask::RegisterMCTrackBranch()
{
   auto *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "The IO manager was not instantiated before AtSimTransportTask::Init().";
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

void AtSimTransportTask::FillMCTracks()
{
   if (fMCTrackArray == nullptr)
      return;

   fMCTrackArray->Clear("C");
   int idx = 0;
   for (auto *particle : fCollector.GetParticles()) {
      new ((*fMCTrackArray)[idx]) AtMCTrack(particle);
      ++idx;
   }
}

void AtSimTransportTask::TransportCurrentEvent(bool beamEvent)
{
   int trackID = 0;
   for (auto *particle : fCollector.GetParticles()) {
      TransportParticle(*particle, trackID, beamEvent);
      ++trackID;
   }
}

void AtSimTransportTask::TransportParticle(const TParticle &particle, int trackID, bool beamEvent)
{
   const int pdgCode = particle.GetPdgCode();
   auto [Z, A] = GetZAFromPDG(pdgCode);
   if (Z == 0 && A == 0)
      return;

   XYZPoint pos(particle.Vx() * kCmToMm, particle.Vy() * kCmToMm, particle.Vz() * kCmToMm);
   PxPyPzEVector mom(particle.Px() * kGeVToMeV, particle.Py() * kGeVToMeV, particle.Pz() * kGeVToMeV,
                     particle.Energy() * kGeVToMeV);

   try {
      if (!IsSensitiveVolume(fSimulation->GetVolumeNameAt(pos)))
         pos = FindSensitiveEntry(pos, mom);

      LOG(info) << "Simulating particle Z=" << Z << " A=" << A << " with initial pos=" << pos << " mm and mom=" << mom
                << " MeV/c";

      // In the generator pipeline, trackID 0 is always the beam particle. Only mark it as a
      // beam track during beam events so the detector can accumulate energy toward a reaction threshold.
      const bool beamTrack = beamEvent && trackID == 0;

      // Submit an initial entering step if starting inside a sensitive volume.
      // Build a synthetic TransportStep with zero energy loss at the start position.
      auto startVolName = fSimulation->GetVolumeNameAt(pos);
      if (IsSensitiveVolume(startVolName)) {
         AtSimTransport::TransportStep initialStep;
         initialStep.pdg = pdgCode;
         initialStep.preVolumeName = startVolName;
         initialStep.postVolumeName = startVolName;
         initialStep.trackMass = mom.M(); // MeV/c^2
         initialStep.prePosition = pos;
         initialStep.postPosition = pos;
         initialStep.preMomentum = mom;
         initialStep.postMomentum = mom;
         if (!SubmitDetectorStep(initialStep, trackID, beamTrack, true, false))
            return;
      }

      fSimulation->TransportParticle(
         Z, A, pos, mom, [this, trackID, beamEvent](const AtSimTransport::TransportStep &step) {
            const bool preSensitive = IsSensitiveVolume(step.preVolumeName);
            const bool postSensitive = IsSensitiveVolume(step.postVolumeName);

            if (!preSensitive && !postSensitive)
               return true; // skip non-sensitive regions

            // Detect volume boundary crossings, not just sensitive/non-sensitive transitions.
            // In Geant4, IsTrackEntering() fires at every volume boundary. We replicate this
            // by also detecting sensitive-to-sensitive transitions (e.g., window -> drift_volume).
            const bool volumeChanged = step.preVolumeName != step.postVolumeName;
            const bool entering = postSensitive && (!preSensitive || volumeChanged);
            const bool exiting = preSensitive && (!postSensitive || volumeChanged);
            const bool currentBeamTrack = beamEvent && trackID == 0;

            // When crossing between two sensitive volumes, split into exit + entry so the
            // detector resets per-volume state (e.g., fELossAcc) at boundaries.
            if (exiting && entering) {
               SubmitDetectorStep(step, trackID, currentBeamTrack, false, true);
               SubmitDetectorStep(step, trackID, currentBeamTrack, true, false);
               return true;
            }

            const bool keepTransporting = SubmitDetectorStep(step, trackID, currentBeamTrack, entering, exiting);
            if (exiting && !postSensitive)
               return false;
            return keepTransporting;
         });
   } catch (const std::invalid_argument &ex) {
      LOG(fatal) << "AtSimTransportTask: skipping particle Z=" << Z << " A=" << A << ": " << ex.what();
   }
}

bool AtSimTransportTask::SubmitDetectorStep(const AtSimTransport::TransportStep &step, int trackID,
                                                bool beamTrack, bool entering, bool exiting)
{
   // When exiting, reference the pre-step state (where the particle was in the volume).
   // Otherwise (entering or inside), reference the post-step state.
   const auto &refPos = exiting ? step.prePosition : step.postPosition;
   const auto &refMom = exiting ? step.preMomentum : step.postMomentum;
   const auto &refVol = exiting ? step.preVolumeName : step.postVolumeName;

   AtTpc::StepState detectorStep;
   detectorStep.trackID = trackID;
   detectorStep.pdg = step.pdg;
   detectorStep.volumeName = refVol.c_str();
   detectorStep.volumeID = kAtTpc;
   detectorStep.detCopyID = 0;
   detectorStep.entering = entering;
   detectorStep.exiting = exiting;
   detectorStep.stopping = !exiting && (step.postMomentum.E() - step.trackMass <= 1e-3);
   detectorStep.disappeared = false;
   detectorStep.energyLoss = step.energyLoss * kMeVToGeV;
   detectorStep.timeNs = 0.;
   detectorStep.trackLength = step.length * kMmToCm;
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

XYZPoint AtSimTransportTask::FindSensitiveEntry(const XYZPoint &pos, const PxPyPzEVector &mom) const
{
   const auto dir = mom.Vect().Unit();
   if (dir.R() == 0.0)
      throw std::invalid_argument("Particle momentum is zero; cannot search for detector entry");

   if (gGeoManager == nullptr)
      throw std::invalid_argument("No TGeoManager available for ray-trace");

   // TGeoManager works in cm; SimpleSim uses mm
   double point_cm[3] = {pos.X() * kMmToCm, pos.Y() * kMmToCm, pos.Z() * kMmToCm};
   double dir_unit[3] = {dir.X(), dir.Y(), dir.Z()};

   // Use TGeo ray-tracing to find the exact boundary crossing into the first sensitive volume.
   // This is faster and more precise than stepping in fixed increments.
   gGeoManager->InitTrack(point_cm, dir_unit);
   constexpr int maxBoundaries = 100;
   for (int i = 0; i < maxBoundaries; ++i) {
      auto *node = gGeoManager->FindNextBoundaryAndStep();
      if (node == nullptr)
         break;
      auto *vol = node->GetVolume();
      if (vol != nullptr && IsSensitiveVolume(vol->GetName())) {
         const double *current = gGeoManager->GetCurrentPoint();
         return XYZPoint(current[0] * kCmToMm, current[1] * kCmToMm, current[2] * kCmToMm);
      }
   }

   throw std::invalid_argument("Particle does not intersect a sensitive detector volume");
}

bool AtSimTransportTask::IsSensitiveVolume(const std::string &volumeName) const
{
   return fDetector != nullptr && fDetector->CheckIfSensitive(volumeName);
}

ClassImp(AtSimTransportTask);
