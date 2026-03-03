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
   EXPECT_NEAR(range1, 133.0, 0.20 * 133.0);   // within 20% of CATIMA
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

TEST_F(AtELossBetheBlochFixture, PionRange)
{
   // Charged pion: q=1, mass=139.57 MeV/c², in Ar at STP density
   AtELossBetheBloch pionModel(1.0, 139.57, 18, 40, 1.65e-3, 188.0);

   double dedx = pionModel.GetdEdx(1.0);
   double range = pionModel.GetRange(1.0);

   EXPECT_GT(dedx, 0.0);
   EXPECT_GT(range, 0.0);
   // Range should be a physically sensible positive number (mm)
   EXPECT_LT(range, 1e6);
}

TEST_F(AtELossBetheBlochFixture, ElectronFormula)
{
   // Electron in H₂: triggers Leo 1994 formula (different from heavy particle formula)
   AtELossBetheBloch eModel(1.0, 0.51099895069, 1, 1, 6.5643e-5, 19.2);

   double dedxElec = eModel.GetdEdx(1.0);
   double dedxProt = model.GetdEdx(1.0); // proton fixture

   EXPECT_GT(dedxElec, 0.0);
   // Electron and proton dE/dx at 1 MeV must differ (different formulas + very different kinematics)
   EXPECT_NE(dedxElec, dedxProt);
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
