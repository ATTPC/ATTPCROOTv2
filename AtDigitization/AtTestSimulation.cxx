#include "AtTestSimulation.h"

#include "AtSimParticleCollector.h"
#include "AtSimpleSimulation.h"

#include <FairLogger.h>
#include <FairMCEventHeader.h>
#include <FairPrimaryGenerator.h>
#include <FairTask.h> // for InitStatus, kSUCCESS

#include <Math/Point3D.h>
#include <Math/Point3Dfwd.h> // for Math, XYZPoint
#include <Math/Vector4D.h>   // for LorentzVector
#include <Math/Vector4Dfwd.h> // for PxPyPzEVector
#include <TDatabasePDG.h>
#include <TParticlePDG.h>

#include <cmath>
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

InitStatus AtTestSimulation::Init()
{
   fSimulation->RegisterBranch();

   if (fPrimGen) {
      // FairPrimaryGenerator::GenerateEvent() requires a non-null FairMCEventHeader.
      fMCHeader = std::make_unique<FairMCEventHeader>();
      fPrimGen->SetEvent(fMCHeader.get());
      fPrimGen->Init();
   }

   return kSUCCESS;
}

void AtTestSimulation::Exec(Option_t *)
{
   fSimulation->NewEvent();

   if (!fPrimGen)
      return;

   fCollector.Clear();
   fPrimGen->GenerateEvent(&fCollector);

   for (const auto &p : fCollector.GetParticles()) {
      auto [Z, A] = GetZAFromPDG(p.pdgCode);
      if (Z == 0 && A == 0) {
         continue; // skip unknown particles
      }

      // FairRoot uses GeV/c for momentum and cm for position; AtSimpleSimulation uses MeV/c and mm.
      XYZPoint pos(p.vx * 10., p.vy * 10., p.vz * 10.); // cm → mm
      PxPyPzEVector mom(p.px * 1000., p.py * 1000., p.pz * 1000., p.e * 1000.); // GeV → MeV

      try {
         LOG(info) << "Simulating particle Z=" << Z << " A=" << A << " with initial pos=" << pos << " mm and mom=" << mom
                   << " MeV/c";
         fSimulation->SimulateParticle(Z, A, pos, mom);
      } catch (const std::invalid_argument &ex) {
         // Particle may start outside the drift volume (e.g. beam upstream) — skip silently
         LOG(debug) << "AtTestSimulation: skipping particle Z=" << Z << " A=" << A << ": " << ex.what();
      }
   }
}

ClassImp(AtTestSimulation);
