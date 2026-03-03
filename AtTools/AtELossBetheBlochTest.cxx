#include "AtELossBetheBloch.h"

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

TEST_F(AtELossBetheBlochFixture, BohrStragglingSanity)
{
   double E0 = 5.0;
   double Ef = model.GetEnergy(E0, 100.0);
   EXPECT_GT(Ef, 0.0);

   double sigma = model.GetElossStraggling(E0, Ef);
   EXPECT_GT(sigma, 0.0);
   EXPECT_LT(sigma, E0); // straggling must be less than total energy
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
