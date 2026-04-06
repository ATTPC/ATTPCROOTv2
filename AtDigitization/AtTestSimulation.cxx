#include "AtTestSimulation.h"

#include "AtDetectorList.h"
#include "AtMCTrack.h"
#include "AtSimParticleCollector.h"
#include "AtSimpleSimulation.h"
#include "AtTpc/AtTpc.h"
#include "AtVertexPropagator.h"

#include <FairLogger.h>
#include <FairMCEventHeader.h>
#include <FairPrimaryGenerator.h>
#include <FairRootManager.h>
#include <FairTask.h> // for InitStatus, kSUCCESS

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for Math, XYZPoint
#include <Math/Vector4D.h>   // for LorentzVector
#include <Math/Vector4Dfwd.h> // for PxPyPzEVector
#include <TClonesArray.h>
#include <TDatabasePDG.h>
#include <TParticlePDG.h>

#include <cmath>
#include <string>
#include <utility>
using namespace ROOT::Math;

// ---------------------------------------------------------------------------
// Helper: extract (Z, A) from a PDG code.
//
// Heavy ions: PDG = 1000000000 + Z*10000 + A*10 + I  (I = isomer level, usually 0)
// Light particles (proton, alpha, etc.): use TDatabasePDG charge/mass.
// ---------------------------------------------------------------------------
namespace {
std::pair<int, int> GetZAFromPDG(int pdg)
{
   if (pdg > 1000000000) {
      int A = (pdg / 10) % 1000;
      int Z = (pdg / 10000) % 1000;
      return {Z, A};
   }
   // Fall back to PDG database
   TParticlePDG *p = TDatabasePDG::Instance()->GetParticle(pdg);
   if (p) {
      // Charge() returns units of |e|/3
      int Z = static_cast<int>(std::round(p->Charge() / 3.0));
      // Mass in GeV/c² → convert to amu (1 amu ≈ 0.9315 GeV/c²)
      int A = static_cast<int>(std::round(p->Mass() / 0.9315));
      return {Z, std::max(A, 1)};
   }
   return {0, 0};
}
} // namespace

void AtTestSimulation::RegisterMCTrackBranch()
{
   auto *ioMan = FairRootManager::Instance();
   if (ioMan == nullptr) {
      LOG(fatal) << "The IO manager was not instantiated before AtTestSimulation::Init().";
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

void AtTestSimulation::FillMCTracks()
{
   if (fMCTrackArray == nullptr)
      return;

   fMCTrackArray->Clear("C");

   for (const auto &p : fCollector.GetParticles()) {
      new ((*fMCTrackArray)[p.trackID]) AtMCTrack(p.pdgCode, -1, p.px, p.py, p.pz, p.vx, p.vy, p.vz, 0.0, 0);
   }
}

InitStatus AtTestSimulation::Init()
{
   if (fDetector == nullptr) {
      LOG(info) << "AtTestSimulation: using standalone AtSimpleSimulation branch writer";
      fSimulation->RegisterBranch();
   } else {
      LOG(info) << "AtTestSimulation: using detector-coupled transport adapter";
   }

   if (fPrimGen) {
      // FairPrimaryGenerator::GenerateEvent() requires a non-null FairMCEventHeader.
      fMCHeader = std::make_unique<FairMCEventHeader>();
      fPrimGen->SetEvent(fMCHeader.get());
      fPrimGen->Init();
   }

   RegisterMCTrackBranch();

   return kSUCCESS;
}

void AtTestSimulation::Exec(Option_t *)
{
   fSimulation->NewEvent();

   if (!fPrimGen)
      return;

   const bool isBeamEvent = AtVertexPropagator::Instance()->IsBeamEvent();
   fCollector.Clear();
   fPrimGen->GenerateEvent(&fCollector);
   FillMCTracks();

   for (const auto &p : fCollector.GetParticles()) {
      auto [Z, A] = GetZAFromPDG(p.pdgCode);
      if (Z == 0 && A == 0) {
         continue; // skip unknown particles
      }

      // FairRoot uses GeV/c for momentum and cm for position; AtSimpleSimulation uses MeV/c and mm.
      XYZPoint pos(p.vx * 10., p.vy * 10., p.vz * 10.); // cm → mm
      PxPyPzEVector mom(p.px * 1000., p.py * 1000., p.pz * 1000., p.e * 1000.); // GeV → MeV

      try {
         if (fDetector != nullptr && !IsSensitiveVolume(fSimulation->GetVolumeNameAt(pos)))
            pos = FindSensitiveEntry(pos, mom);

         LOG(info) << "Simulating particle Z=" << Z << " A=" << A << " with initial pos=" << pos << " mm and mom=" << mom
                   << " MeV/c";
         if (fDetector != nullptr) {
            const bool beamTrack = isBeamEvent && p.trackID == 0;
            if (IsSensitiveVolume(fSimulation->GetVolumeNameAt(pos)) &&
                !SubmitInitialSensitivePoint(p.trackID, p.pdgCode, beamTrack, pos, mom))
               continue;
            bool seenSensitiveVolume = false;
            fSimulation->TransportParticle(
               Z, A, pos, mom,
               [this, trackID = p.trackID, isBeamEvent, seenSensitiveVolume](const AtSimpleSimulation::TransportStep &step
                                                                             ) mutable {
                  const bool preSensitive = seenSensitiveVolume || IsSensitiveVolume(step.preVolumeName);
                  const bool postSensitive = IsSensitiveVolume(step.postVolumeName);
                  const bool entering = !seenSensitiveVolume && postSensitive;
                  const bool exiting = seenSensitiveVolume && !postSensitive;
                  const bool isBeamTrack = isBeamEvent && trackID == 0;
                  if (postSensitive)
                     seenSensitiveVolume = true;
                  return ProcessDetectorStep(step, trackID, isBeamTrack, preSensitive, postSensitive, entering, exiting);
               });
         } else {
            fSimulation->SimulateParticle(Z, A, pos, mom);
         }
      } catch (const std::invalid_argument &ex) {
         // Legacy direct simulation only supports drift-volume starts. The detector-coupled path
         // also rejects tracks that begin outside the imported geometry.
         LOG(debug) << "AtTestSimulation: skipping particle Z=" << Z << " A=" << A << ": " << ex.what();
      }
   }
}

bool AtTestSimulation::SubmitInitialSensitivePoint(int trackID, int pdg, bool beamTrack, const XYZPoint &pos,
                                                   const PxPyPzEVector &mom)
{
   if (fDetector == nullptr)
      return true;

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
   detectorStep.totalEnergy = mom.E() / 1000.;
   detectorStep.trackMass = mom.M() / 1000.;
   detectorStep.pos.SetXYZT(pos.X() / 10., pos.Y() / 10., pos.Z() / 10., 0.0);
   detectorStep.mom.SetXYZT(mom.Px() / 1000., mom.Py() / 1000., mom.Pz() / 1000., mom.E() / 1000.);
   detectorStep.posOut = detectorStep.pos;
   detectorStep.momOut = detectorStep.mom;

   return !fDetector->ProcessStep(detectorStep);
}

bool AtTestSimulation::ProcessDetectorStep(const AtSimpleSimulation::TransportStep &step, int trackID, bool beamTrack,
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
   detectorStep.energyLoss = step.energyLoss / 1000.;
   detectorStep.timeNs = 0.;
   detectorStep.trackLength = step.length / 10.;

   const auto &refPos = postSensitive ? step.postPosition : step.prePosition;
   const auto &refMom = postSensitive ? step.postMomentum : step.preMomentum;
   detectorStep.totalEnergy = refMom.E() / 1000.;
   detectorStep.trackMass = step.trackMass / 1000.;
   detectorStep.pos.SetXYZT(refPos.X() / 10., refPos.Y() / 10., refPos.Z() / 10., 0.);
   detectorStep.mom.SetXYZT(refMom.Px() / 1000., refMom.Py() / 1000., refMom.Pz() / 1000., refMom.E() / 1000.);
   detectorStep.posOut.SetXYZT(step.postPosition.X() / 10., step.postPosition.Y() / 10., step.postPosition.Z() / 10., 0.);
   detectorStep.momOut.SetXYZT(step.postMomentum.Px() / 1000., step.postMomentum.Py() / 1000., step.postMomentum.Pz() / 1000.,
                               step.postMomentum.E() / 1000.);

   const bool stopTransport = fDetector->ProcessStep(detectorStep);
   return !stopTransport;
}

XYZPoint AtTestSimulation::FindSensitiveEntry(const XYZPoint &pos, const PxPyPzEVector &mom) const
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

bool AtTestSimulation::IsSensitiveVolume(const std::string &volumeName)
{
   return volumeName.find("drift_volume") != std::string::npos || volumeName.find("window") != std::string::npos ||
          volumeName.find("cell") != std::string::npos;
}

ClassImp(AtTestSimulation);
