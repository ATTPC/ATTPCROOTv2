/**
 * Unit tests for AtSimpleSimulation.
 *
 * Two physics tests:
 *  1. ZeroFieldStraightLine  — zero E/B fields produce a collinear track along the
 *     initial momentum direction.  Total energy loss matches the analytic integral
 *     of a constant-dEdx model.
 *
 *  2. MagneticFieldLarmorRadius — a non-zero B field along Z curves a transverse
 *     proton into a circle whose radius matches the relativistic Larmor formula
 *     r = p⊥ / (q B).
 *
 * No external files are used; geometry and energy-loss model are built in memory.
 */

#include "AtMCPoint.h"
#include "AtSimpleSimulation.h"
#define private public
#define protected public
#include "AtTestSimulation.h"
#undef protected
#undef private

#include "AtELossModel.h"
#include "AtMCTrack.h"
#include "AtTpc/AtTpc.h"
#include "AtVertexPropagator.h"

#include <Math/Point3D.h>
#include <Math/Vector4D.h>
#include <TClonesArray.h>
#include <TGeoManager.h>
#include <TGeoMaterial.h>
#include <TGeoMedium.h>
#include <TGeoVolume.h>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Minimal energy-loss model with constant dEdx = fRate [MeV/mm].
// ---------------------------------------------------------------------------
class ConstELoss : public AtTools::AtELossModel {
public:
   double fRate;
   explicit ConstELoss(double rate = 1.0) : AtTools::AtELossModel(0), fRate(rate) {}

   double GetdEdx(double /*KE*/) const override { return fRate; }
   double GetRange(double ei, double ef = 0) const override
   {
      return (fRate > 0) ? (ei - ef) / fRate : 1e9;
   }
   double GetEnergyLoss(double /*KE*/, double dist) const override { return fRate * dist; }
   double GetEnergy(double ei, double dist) const override { return std::max(0.0, ei - fRate * dist); }
   double GetElossStraggling(double, double) const override { return 0; }
   double GetdEdxStraggling(double, double) const override { return 0; }
   double GetRangeVariance(double) const override { return 0; }
};

// ---------------------------------------------------------------------------
// Fixture: builds an in-memory TGeoManager with a 100×100×100 cm "cave"
// containing a 50×50×50 cm "drift_volume" box.  AtSimpleSimulation calls
// gGeoManager->FindNode() to determine whether a position is inside the
// active volume, so we need this geometry even in unit tests.
// ---------------------------------------------------------------------------
class AtSimTest : public ::testing::Test {
protected:
   void SetUp() override
   {
      if (gGeoManager != nullptr)
         delete gGeoManager;

      new TGeoManager("test_geo", "test geometry");
      auto *mat = new TGeoMaterial("Vacuum", 0, 0, 0);
      auto *med = new TGeoMedium("Vacuum", 1, mat);
      auto *top = gGeoManager->MakeBox("cave", med, 100, 100, 100); // 100 cm half-widths
      gGeoManager->SetTopVolume(top);
      auto *drift = gGeoManager->MakeBox("drift_volume", med, 50, 50, 50); // 50 cm = 500 mm
      top->AddNode(drift, 1);
      gGeoManager->CloseGeometry();
   }
};

// ---------------------------------------------------------------------------
// Test 1 — ZeroFieldStraightLine
//
// Fire a proton along +Z (no E or B field).  With a constant 1 MeV/mm model
// and KE₀ = 50 MeV, the particle stops after 50 mm.
// ---------------------------------------------------------------------------
TEST_F(AtSimTest, ZeroFieldStraightLine)
{
   AtSimpleSimulation sim;
   sim.AddModel(1, 1, std::make_shared<ConstELoss>(1.0 /*MeV/mm*/));

   // Proton: KE = 50 MeV → p_z ≈ 310.5 MeV/c, E ≈ 988.3 MeV
   const double mass_p = 938.272; // MeV/c²
   const double KE0 = 50.0;      // MeV
   const double E0 = mass_p + KE0;
   const double p0 = std::sqrt(E0 * E0 - mass_p * mass_p);

   ROOT::Math::XYZPoint pos(0, 0, 0);                  // mm
   ROOT::Math::PxPyPzEVector mom(0.0, 0.0, p0, E0);   // MeV

   sim.NewEvent();
   sim.SimulateParticle(1, 1, pos, mom);

   int nPts = sim.GetNumPoints();
   ASSERT_GT(nPts, 0) << "No MCPoints were produced";

   // --- Check each point lies on the Z axis ---
   double totalELoss = 0;
   double prevZ = -1e9;
   for (int i = 0; i < nPts; ++i) {
      auto *pt = dynamic_cast<AtMCPoint *>(sim.GetPointsArray().At(i));
      ASSERT_NE(pt, nullptr);

      // X and Y must be (almost) zero — positions stored in cm
      EXPECT_NEAR(pt->GetX(), 0.0, 1e-9) << "Point " << i << " X ≠ 0";
      EXPECT_NEAR(pt->GetY(), 0.0, 1e-9) << "Point " << i << " Y ≠ 0";

      // Z must be monotonically increasing
      double z = pt->GetZ();
      EXPECT_GT(z, prevZ) << "Z not monotonically increasing at point " << i;
      prevZ = z;

      totalELoss += pt->GetEnergyLoss() * 1000.; // GeV → MeV
   }

   // --- Last point must be near the expected stopping position (50 mm = 5 cm) ---
   auto *last = dynamic_cast<AtMCPoint *>(sim.GetPointsArray().At(nPts - 1));
   EXPECT_NEAR(last->GetZ(), 5.0 /*cm*/, 0.2) << "Particle didn't stop near expected range";

   // --- Total energy loss must equal KE₀ within 5% ---
   EXPECT_NEAR(totalELoss, KE0, KE0 * 0.05) << "Total energy loss differs from initial KE";
}

// ---------------------------------------------------------------------------
// Test 2 — MagneticFieldLarmorRadius
//
// Fire a proton with purely transverse momentum p_x in a B = (0,0,2 T) field.
// The proton should trace a circle in the XY plane with the relativistic
// Larmor radius r = p⊥ / (q B).
//
// Expected radius (SI calculation):
//   p⊥  = 100 MeV/c = 100 × 1.60218e-13 / 299792458 kg·m/s ≈ 5.344e-20 kg·m/s
//   q   = 1.60218e-19 C
//   B   = 2 T
//   r   = p⊥ / (q B) ≈ 0.1669 m ≈ 166.9 mm
// ---------------------------------------------------------------------------
TEST_F(AtSimTest, MagneticFieldLarmorRadius)
{
   AtSimpleSimulation sim;
   // Tiny energy-loss rate keeps KE almost constant (avoids infinite loop in
   // straight-line path, irrelevant here since B≠0 uses AtPropagator).
   sim.AddModel(1, 1, std::make_shared<ConstELoss>(0.0 /*MeV/mm — no drag*/));
   // Use explicit XYZVector construction to ensure the B field is recognised as non-zero.
   sim.SetMagneticField(ROOT::Math::XYZVector(0., 0., 2.0)); // 2 T along Z

   const double mass_p = 938.272;       // MeV/c²
   const double px0 = 100.0;           // MeV/c (purely transverse)
   const double E0 = std::sqrt(px0 * px0 + mass_p * mass_p);

   ROOT::Math::XYZPoint pos(0, 0, 0);
   ROOT::Math::PxPyPzEVector mom(px0, 0.0, 0.0, E0); // MeV

   // Stop after 200 steps — enough to trace ≈ one full Larmor circle
   int stepCount = 0;
   const int maxSteps = 200;
   auto stopFunc = [&stepCount, maxSteps](ROOT::Math::XYZPoint, ROOT::Math::PxPyPzEVector) -> bool {
      return ++stepCount < maxSteps;
   };

   sim.NewEvent();
   sim.SimulateParticle(1, 1, pos, mom, stopFunc);

   int nPts = sim.GetNumPoints();
   ASSERT_GT(nPts, 5) << "Too few MCPoints for Larmor test (got " << nPts << ")";

   // Sanity check: if the curved path is active, the proton must curve in -Y.
   // If all Y-coordinates are ~0, the simulation used the straight-line path (B field inactive).
   {
      auto *pt0 = dynamic_cast<AtMCPoint *>(sim.GetPointsArray().At(nPts / 2));
      ASSERT_NE(pt0, nullptr);
      EXPECT_NE(pt0->GetY(), 0.0) << "Y=0 at midpoint: B field not activating curved path";
   }

   // Analytic Larmor radius in mm
   // r [m] = p[kg·m/s] / (q[C] * B[T])
   // p [MeV/c] → [kg·m/s] via 1 MeV/c = 1.60218e-13 J / 299792458 m/s
   const double MeV_per_c_to_SI = 1.60218e-13 / 299792458.0;
   const double q = 1.60218e-19; // C
   const double B = 2.0;         // T
   const double p_SI = px0 * MeV_per_c_to_SI;
   const double larmor_mm = p_SI / (q * B) * 1000.; // m → mm

   // Initial momentum is in +X; B is in +Z.
   // Force F = q(v × B): v = vx*x̂, B = Bz*ẑ  →  v×B = vx*Bz*(x̂×ẑ) = -vx*Bz*ŷ
   // The proton curves in the -Y direction; circle center is at (0, -r, 0).
   const double cx = 0.0;
   const double cy = -larmor_mm;

   // Check that every MCPoint lies on the expected circle (within 5%)
   double sumErr = 0;
   for (int i = 0; i < nPts; ++i) {
      auto *pt = dynamic_cast<AtMCPoint *>(sim.GetPointsArray().At(i));
      ASSERT_NE(pt, nullptr);

      double x_mm = pt->GetX() * 10.; // cm → mm
      double y_mm = pt->GetY() * 10.;
      double dx = x_mm - cx;
      double dy = y_mm - cy;
      double r = std::sqrt(dx * dx + dy * dy);
      sumErr += std::abs(r - larmor_mm);
   }
   double meanErr = sumErr / nPts;
   EXPECT_LT(meanErr, larmor_mm * 0.05)
      << "Mean Larmor radius error " << meanErr << " mm exceeds 5% of " << larmor_mm << " mm";
}

TEST_F(AtSimTest, LegacySimulateParticleStillRejectsStartsOutsideDriftVolume)
{
   AtSimpleSimulation sim;
   sim.AddModel(1, 1, std::make_shared<ConstELoss>(0.1));

   const double mass_p = 938.272;
   const double E0 = mass_p + 10.0;
   const double p0 = std::sqrt(E0 * E0 - mass_p * mass_p);

   ROOT::Math::XYZPoint pos(0, 0, 600.0); // Outside drift_volume but still in cave
   ROOT::Math::PxPyPzEVector mom(0.0, 0.0, -p0, E0);

   sim.NewEvent();
   EXPECT_THROW(sim.SimulateParticle(1, 1, pos, mom), std::invalid_argument);
}

TEST_F(AtSimTest, TransportParticleInvokesCallbackAcrossVolumeBoundary)
{
   AtSimpleSimulation sim;
   sim.AddModel(1, 1, std::make_shared<ConstELoss>(0.0));
   sim.SetDistanceStep(10.0);

   const double mass_p = 938.272;
   const double E0 = mass_p + 10.0;
   const double p0 = std::sqrt(E0 * E0 - mass_p * mass_p);

   ROOT::Math::XYZPoint pos(0, 0, -600.0); // In cave, upstream of drift_volume
   ROOT::Math::PxPyPzEVector mom(0.0, 0.0, p0, E0);

   bool sawCaveToDrift = false;
   int callbackCount = 0;

   sim.NewEvent();
   sim.TransportParticle(1, 1, pos, mom, [&](const AtSimpleSimulation::TransportStep &step) {
      ++callbackCount;
      if (step.preVolumeName == "cave" && step.postVolumeName == "drift_volume")
         sawCaveToDrift = true;
      return callbackCount < 30;
   });

   EXPECT_GT(callbackCount, 0);
   EXPECT_TRUE(sawCaveToDrift);
   EXPECT_EQ(sim.GetNumPoints(), 0) << "Detector-coupled transport should not emit legacy MC points";
}

TEST_F(AtSimTest, ReactionMCTracksKeepGeneratedTrackIDs)
{
   auto sim = std::make_unique<AtSimpleSimulation>();
   AtTestSimulation task(std::move(sim));
   task.fMCTrackArray = new TClonesArray("AtMCTrack");

   Int_t ntr = -1;
   task.fCollector.PushTrack(1, -1, 2212, 0.1, 0.0, 0.2, 0.95, 0.0, 0.0, 14.0, 0.0, 0.0, 0.0, 0.0, kPPrimary, ntr,
                             1.0, 0, -1);
   task.fCollector.PushTrack(1, -1, 1000020040, 0.0, 0.0, 0.3, 3.8, 0.0, 0.0, 14.0, 0.0, 0.0, 0.0, 0.0, kPPrimary,
                             ntr, 1.0, 0, -1);

   task.FillMCTracks();

   ASSERT_EQ(task.fMCTrackArray->GetEntriesFast(), 2);

   auto *proton = dynamic_cast<AtMCTrack *>(task.fMCTrackArray->At(0));
   auto *alpha = dynamic_cast<AtMCTrack *>(task.fMCTrackArray->At(1));

   ASSERT_NE(proton, nullptr);
   ASSERT_NE(alpha, nullptr);

   EXPECT_EQ(proton->GetPdgCode(), 2212);
   EXPECT_EQ(proton->GetMotherId(), -1);
   EXPECT_EQ(alpha->GetPdgCode(), 1000020040);
   EXPECT_EQ(alpha->GetMotherId(), -1);
}

TEST_F(AtSimTest, BeamMCTracksKeepBeamAtTrackZero)
{
   auto sim = std::make_unique<AtSimpleSimulation>();
   AtTestSimulation task(std::move(sim));
   task.fMCTrackArray = new TClonesArray("AtMCTrack");

   Int_t ntr = -1;
   task.fCollector.PushTrack(1, -1, 2212, 0.0, 0.0, 0.043, 0.94, 0.0, 0.0, -10.0, 0.0, 0.0, 0.0, 0.0, kPPrimary, ntr,
                             1.0, 0, -1);

   task.FillMCTracks();

   ASSERT_EQ(task.fMCTrackArray->GetEntriesFast(), 1);
   auto *beam = dynamic_cast<AtMCTrack *>(task.fMCTrackArray->At(0));
   ASSERT_NE(beam, nullptr);
   EXPECT_EQ(beam->GetPdgCode(), 2212);
   EXPECT_EQ(beam->GetMotherId(), -1);
}

TEST_F(AtSimTest, InitialSensitivePointUsesTrackStartState)
{
   auto sim = std::make_unique<AtSimpleSimulation>();
   AtTestSimulation task(std::move(sim));
   AtTpc detector;
   task.fDetector = &detector;

   AtVertexPropagator::Instance()->ResetForTesting();
   AtVertexPropagator::Instance()->SetBeamMass(16.014701);
   AtVertexPropagator::Instance()->ResetVertex();

   const double mass = 16.014701 * 931.494;
   ROOT::Math::XYZPoint pos(0.0, 0.0, 1.0);
   ROOT::Math::PxPyPzEVector mom(0.0, 0.0, 2297.0, std::sqrt(2297.0 * 2297.0 + mass * mass));

   const bool keepTransporting = task.SubmitInitialSensitivePoint(0, 1000060160, true, pos, mom);

   EXPECT_TRUE(keepTransporting);
   auto *points = detector.GetCollection(0);
   ASSERT_NE(points, nullptr);
   ASSERT_EQ(points->GetEntriesFast(), 1);

   auto *point = dynamic_cast<AtMCPoint *>(points->At(0));
   ASSERT_NE(point, nullptr);
   EXPECT_EQ(point->GetTrackID(), 0);
   EXPECT_NEAR(point->GetZ() * 10., 1.0, 1e-9);
   EXPECT_NEAR(point->GetLength() * 10., 0.0, 1e-9);
   EXPECT_NEAR(point->GetEnergyLoss() * 1e6, 0.0, 1e-9);
}
