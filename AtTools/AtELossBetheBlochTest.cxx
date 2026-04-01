#include "AtELossBetheBloch.h"

#include "AtELossCATIMA.h"

#include <catima/catima.h>
#include <catima/config.h>
#include <cmath>
#include <gtest/gtest.h>

using namespace AtTools;

// Proton rest mass in MeV/c²
static constexpr double kProtonMass = 938.272;

/**
 * Test fixture: proton in H₂ gas at 600 Torr (density 6.5643e-5 g/cm³, I = 19.2 eV).
 * CATIMA/LISE reference ranges: 133 mm at 1 MeV, 7888 mm at 10 MeV.
 */
class AtELossBetheBlochFixture : public ::testing::Test {
protected:
   AtELossBetheBloch model;

   AtELossBetheBlochFixture() : model(1.0, kProtonMass, 1, 1, 6.5643e-5, 19.2) {}
};

TEST_F(AtELossBetheBlochFixture, ConstructModel)
{
   EXPECT_GT(model.GetdEdx(1.0), 0.0);
}

TEST_F(AtELossBetheBlochFixture, ProtonRange_vs_SRIM)
{
   // CATIMA/LISE reference: 133 mm at 1 MeV, 7888 mm at 10 MeV in H₂ at 6.5643e-5 g/cm³
   double range1 = model.GetRange(1.0);
   double range10 = model.GetRange(10.0);

   EXPECT_GT(range1, 0.0);
   EXPECT_GT(range10, 0.0);
   EXPECT_NEAR(range1, 133.0, 0.20 * 133.0);    // within 20% of CATIMA
   EXPECT_NEAR(range10, 7888.0, 0.20 * 7888.0); // within 20% of CATIMA
}

TEST_F(AtELossBetheBlochFixture, ProtonEnergyLoss)
{
   double distance = 100.0; // mm
   double eFinal = model.GetEnergy(10.0, distance);

   EXPECT_GT(eFinal, 0.0);
   EXPECT_LT(eFinal, 10.0);

   // Self-consistency: range from 10 MeV to eFinal must equal the distance we traveled
   double rangeCheck = model.GetRange(10.0, eFinal);
   EXPECT_NEAR(rangeCheck, distance, 0.01 * distance);
}

TEST_F(AtELossBetheBlochFixture, AlphaParticle)
{
   // 4He: q=2, mass = 4.00260325413 amu * 931.494 MeV/amu = 3727.38 MeV/c²
   AtELossBetheBloch alphaModel(2.0, 3727.38, 1, 1, 6.5643e-5, 19.2);

   // CATIMA/LISE reference: 657 mm at 10 MeV in H₂ at 6.5643e-5 g/cm³
   double range10 = alphaModel.GetRange(10.0);
   EXPECT_GT(range10, 0.0);
   EXPECT_NEAR(range10, 657.0, 0.20 * 657.0); // within 20% of CATIMA
}

/**
 * Charged pion (π⁺): q=1, mass=139.57 MeV/c² in Ar gas (Z=18, A=40, ρ=1.65e-3 g/cm³, I=188 eV).
 *
 * Expected dEdx computed analytically from PDG Bethe-Bloch (Eq. 34.1):
 *
 *   T=1 MeV:  β²=0.01418, Tmax=0.01459 MeV, logArg=6069
 *             ½ln(6069) − β² = 4.341
 *             −dE/dx = 0.307075 × 0.45 × 1.65e-3 / 0.01418 × 4.341 = 6.98e-3 MeV/mm
 *
 *   T=10 MeV: β²=0.12930, Tmax=0.15059 MeV, logArg=646570
 *             ½ln(646570) − β² = 6.561
 *             −dE/dx = 0.307075 × 0.45 × 1.65e-3 / 0.12930 × 6.561 = 1.157e-3 MeV/mm
 */
TEST_F(AtELossBetheBlochFixture, PiondEdx)
{
   AtELossBetheBloch pionModel(1.0, 139.57, 18, 40, 1.65e-3, 188.0);

   // Analytic Bethe-Bloch values, tolerance 2% (accounts for spline interpolation)
   EXPECT_NEAR(pionModel.GetdEdx(1.0), 6.98e-3, 0.02 * 6.98e-3);
   EXPECT_NEAR(pionModel.GetdEdx(10.0), 1.157e-3, 0.02 * 1.157e-3);

   // dEdx must fall with increasing energy in this non-relativistic regime
   EXPECT_GT(pionModel.GetdEdx(1.0), pionModel.GetdEdx(10.0));

   // Self-consistency: traveling to half-range and back gives the right energy
   double halfRange = pionModel.GetRange(10.0) / 2.0;
   double eMid = pionModel.GetEnergy(10.0, halfRange);
   EXPECT_NEAR(pionModel.GetRange(10.0, eMid), halfRange, 0.01 * halfRange);
}

/**
 * Electron in H₂ (same material as fixture): triggers the Leo 1994 modified formula.
 *
 * Expected dEdx from Leo 1994 Eq. 2.38 (F⁻ = Møller exchange correction):
 *
 *   T=1 MeV:  τ=1.957, β²=0.8858, F⁻=−0.2204, logArg=73268
 *             −dE/dx = 0.307075 × 1 × 6.5643e-5 / 0.8858 × (ln(73268) − 0.2204)
 *                    = 2.499e-5 MeV/mm
 *
 *   T=5 MeV:  τ=9.786, β²=0.9915, F⁻=−0.01107, logArg=632450
 *             −dE/dx = 0.307075 × 1 × 6.5643e-5 / 0.9915 × (ln(632450) − 0.01107)
 *                    = 2.714e-5 MeV/mm
 *
 * The minimum of electron stopping power occurs near 1 MeV (minimum-ionizing point),
 * so dEdx(1 MeV) < dEdx(5 MeV) and dEdx(1 MeV) < dEdx(0.5 MeV).
 */
TEST_F(AtELossBetheBlochFixture, ElectrondEdx)
{
   AtELossBetheBloch eModel(1.0, 0.51099895069, 1, 1, 6.5643e-5, 19.2);

   // Analytic Leo 1994 values, tolerance 2%
   EXPECT_NEAR(eModel.GetdEdx(1.0), 2.499e-5, 0.02 * 2.499e-5);
   EXPECT_NEAR(eModel.GetdEdx(5.0), 2.714e-5, 0.02 * 2.714e-5);

   // Electron minimum-ionizing: dEdx has a minimum near 1 MeV
   // → dEdx rises on both sides of the minimum
   EXPECT_GT(eModel.GetdEdx(0.5), eModel.GetdEdx(1.0)); // falling towards minimum
   EXPECT_GT(eModel.GetdEdx(5.0), eModel.GetdEdx(1.0)); // rising away from minimum

   // Electron formula gives different result from the heavy-particle proton formula
   EXPECT_GT(std::abs(eModel.GetdEdx(1.0) - model.GetdEdx(1.0)) / model.GetdEdx(1.0), 0.01);
}

TEST_F(AtELossBetheBlochFixture, SettersRebuildSpline)
{
   // Start with proton in H₂ (same as fixture)
   AtELossBetheBloch m(1.0, kProtonMass, 1, 1, 6.5643e-5, 19.2);
   double dedx_h2 = m.GetdEdx(1.0);

   // SetMaterial: switch to Ar — dEdx must change immediately without calling BuildSpline manually
   m.SetMaterial(18, 40, 1.65e-3, 188.0);
   EXPECT_NE(m.GetdEdx(1.0), dedx_h2);

   // SetDensity: double the Ar density — dEdx must scale proportionally
   double dedx_ar = m.GetdEdx(1.0);
   m.SetDensity(2.0 * 1.65e-3);
   EXPECT_NEAR(m.GetdEdx(1.0), 2.0 * dedx_ar, 0.01 * dedx_ar);

   // SetI: restore original density, then change I — dEdx must change
   m.SetDensity(1.65e-3);
   double dedx_before = m.GetdEdx(1.0);
   m.SetI(100.0); // different I value
   EXPECT_NE(m.GetdEdx(1.0), dedx_before);
}

// Proton in Ar gas (same material as PiondEdx test).
class AtELossBetheBlochArFixture : public ::testing::Test {
protected:
   AtELossBetheBloch model;
   // Ar: Z=18, A=40, ρ=1.65e-3 g/cm³, I=188 eV
   AtELossBetheBlochArFixture() : model(1.0, kProtonMass, 18, 40, 1.65e-3, 188.0) {}
};

TEST_F(AtELossBetheBlochArFixture, RangeVarianceZeroAtLowEnergy)
{
   EXPECT_DOUBLE_EQ(model.GetRangeVariance(0.0), 0.0);
   EXPECT_DOUBLE_EQ(model.GetRangeVariance(1e-6), 0.0);
}

TEST_F(AtELossBetheBlochArFixture, RangeVarianceMonotonicallyIncreasing)
{
   double prev = 0.0;
   for (double E : {1.0, 2.0, 5.0, 10.0, 20.0, 50.0}) {
      double rv = model.GetRangeVariance(E);
      EXPECT_GT(rv, prev) << "RangeVariance not increasing at E=" << E;
      prev = rv;
   }
}

TEST_F(AtELossBetheBlochArFixture, StragglingScalesWithSqrtRange)
{
   // For a factor-4 path length ratio, straggling should scale as sqrt(path) → factor ~2.
   // Use two different path lengths by picking final energies via GetEnergy.
   double E0 = 20.0;
   double Ef1 = model.GetEnergy(E0, 500.0);
   double Ef2 = model.GetEnergy(E0, 2000.0); // 4× longer path
   ASSERT_GT(Ef1, 0.0);
   ASSERT_GT(Ef2, 0.0);

   double sigma1 = model.GetElossStraggling(E0, Ef1);
   double sigma2 = model.GetElossStraggling(E0, Ef2);
   ASSERT_GT(sigma1, 0.0);
   ASSERT_GT(sigma2, 0.0);

   // σ scales roughly as sqrt(Δx), so σ2/σ1 ≈ sqrt(4) = 2.
   // Allow 20% tolerance: exact scaling only holds when dEdx is constant over the path.
   double ratio = sigma2 / sigma1;
   EXPECT_NEAR(ratio, 2.0, 0.20 * 2.0);
}

TEST_F(AtELossBetheBlochArFixture, dEdxStragglingConsistency)
{
   // GetdEdxStraggling * GetRange must equal GetElossStraggling (up to floating point).
   double E0 = 10.0;
   double Ef = model.GetEnergy(E0, 1000.0);
   ASSERT_GT(Ef, 0.0);

   double sigmaE = model.GetElossStraggling(E0, Ef);
   double dx = model.GetRange(E0, Ef);
   double sigmaDedx = model.GetdEdxStraggling(E0, Ef);

   ASSERT_GT(sigmaE, 0.0);
   ASSERT_GT(dx, 0.0);
   EXPECT_NEAR(sigmaDedx * dx, sigmaE, 1e-9 * sigmaE);
}

TEST_F(AtELossBetheBlochArFixture, StragglingBroadRange)
{
   // Physical bound: straggling cannot exceed energy loss (σ(ΔE) < ΔE).
   double E0 = 10.0;
   double Ef = 5.0;
   double dE = E0 - Ef;
   double sigma = model.GetElossStraggling(E0, Ef);

   EXPECT_GT(sigma, 0.0);
   EXPECT_LT(sigma, dE);
   // Also check the ratio is sub-50% (physically meaningful bound)
   EXPECT_LT(sigma / dE, 0.5);
}

TEST_F(AtELossBetheBlochFixture, BlochApprox)
{
   // Default I (uses Bloch approx I ≈ 13.5 * Z eV = 13.5 eV for hydrogen)
   AtELossBetheBloch blochModel(1.0, kProtonMass, 1, 1, 6.5643e-5);

   double rangeBloch = blochModel.GetRange(1.0);
   double rangeExact = model.GetRange(1.0); // uses I=19.2 eV

   EXPECT_GT(rangeBloch, 0.0);
   // Bloch approximation should agree to within 30% of the explicit I value
   EXPECT_NEAR(rangeBloch, rangeExact, 0.30 * rangeExact);
}

/**
 * Benchmark fixture: compare AtELossBetheBloch against AtELossCATIMA configured for pure Bohr
 * straggling (z_eff_type::none = bare charge, default calculation = bohr mode).
 *
 * System: proton in H₂ at 600 Torr (ρ = 6.5643e-5 g/cm³), identical to AtELossCATIMATestFixture.
 * Expected agreement: within 10% (residual difference from Lindhard X × γ² ≈ 1 for protons at 1–10 MeV).
 */
class AtELossBetheBlochVsCATIMAFixture : public ::testing::Test {
protected:
   static constexpr double kProtonMassAmu = 1.007825031898;
   static constexpr double kDensity = 6.5643e-5; // g/cm³

   AtTools::AtELossBetheBloch bb;
   AtTools::AtELossCATIMA catima;

   AtELossBetheBlochVsCATIMAFixture()
      : bb(1.0, kProtonMass, 1, 1, kDensity, 19.2), catima(kDensity, catima::Material(1, 1))
   {
      catima.SetProjectile(1, 1, kProtonMassAmu);

      // Configure CATIMA to pure Bohr: bare charge (no effective-charge model),
      // default calculation=bohr already disables the ATIMA correction.
      catima::Config pureBohrCfg;
      pureBohrCfg.z_effective = catima::z_eff_type::none;
      catima.SetConfig(pureBohrCfg);
   }
};

TEST_F(AtELossBetheBlochVsCATIMAFixture, CATIMAComparison_RangeVariance)
{
   // Compare accumulated Bohr range variance Ω²(E) [mm²] at three energies.
   // Both models implement the same Bohr formula; agreement within 10% is expected.
   for (double E : {1.0, 5.0, 10.0}) {
      double bbVal = bb.GetRangeVariance(E);
      double catimaVal = catima.GetRangeVariance(E);
      ASSERT_GT(bbVal, 0.0) << "BB range variance zero at E=" << E;
      ASSERT_GT(catimaVal, 0.0) << "CATIMA range variance zero at E=" << E;
      EXPECT_NEAR(bbVal, catimaVal, 0.10 * catimaVal) << "Range variance mismatch at E=" << E << " MeV";
   }
}

TEST_F(AtELossBetheBlochVsCATIMAFixture, CATIMAComparison_EnergyStraggling)
{
   // Reference energy pairs from AtELossCATIMATestFixture::TestEnergyLossStraggling.
   // CATIMA (pure Bohr config) is used as the reference; BB must agree within 10%.
   const double mass = kProtonMassAmu;
   struct Case {
      double eIni, eFin;
   };
   const Case cases[] = {
      {1.0, 0.75 * mass},    // narrow: ~25% energy loss
      {5.0, 3.58164 * mass}, // narrow: ~28% energy loss
      {5.0, 1.0},            // wide: 80% energy loss
      {10.0, 1.0},           // wide: 90% energy loss, exercises Bragg-peak region
   };
   for (auto &c : cases) {
      double bbSigma = bb.GetElossStraggling(c.eIni, c.eFin);
      double catimaSigma = catima.GetElossStraggling(c.eIni, c.eFin);
      ASSERT_GT(bbSigma, 0.0) << "BB straggling zero for E0=" << c.eIni;
      ASSERT_GT(catimaSigma, 0.0) << "CATIMA straggling zero for E0=" << c.eIni;
      EXPECT_NEAR(bbSigma, catimaSigma, 0.10 * catimaSigma) << "Energy straggling mismatch for E0=" << c.eIni << " MeV";
   }
}
