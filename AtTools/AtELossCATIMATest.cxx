#include "AtELossCATIMA.h"

#include <FairLogger.h>

#include <catima/catima.h>
#include <catima/structures.h>
#include <cmath>
#include <gtest/gtest.h>
#include <tuple>
#include <utility>
#include <vector>

/**
 * Test fixture for the AtELossCATIMA class that initializes a H2 gas w/ proton model.
 */
class AtELossCATIMATestFixture : public ::testing::Test {
protected:
   AtTools::AtELossCATIMA model;
   double mass{1.007825031898}; // Mass of proton in amu

   AtELossCATIMATestFixture() : model(6.5643e-5)
   {
      // Initialize the CATIMA model with H2 gas density and components and set projectile to proton.
      model.SetMaterial(catima::Material(1, 1)); // Set material to H2
      model.SetProjectile(1, 1, mass);           // Set projectile to proton
   }
};

TEST_F(AtELossCATIMATestFixture, ConstructCATIMAModel)
{
   // Basic check: ensure model is constructed and can compute range
   double range = model.GetRange(1.0); // 1 MeV
   EXPECT_GT(range, 0.0);
}

TEST_F(AtELossCATIMATestFixture, TestRangeStraggling)
{
   // Check dEdx for a known energy
   double range_var = model.GetRangeVariance(1.0); // 1 MeV
   double expected = 1.99;                         // Expected value from LISE for H2 at 1 MeV
   ASSERT_NEAR(range_var, expected * expected, 0.2 * expected * expected);

   double range_straggling = model.GetRangeStraggling(1.0); // 1 MeV
   ASSERT_NEAR(range_straggling, expected, 0.1 * expected);

   // Check straggling at 10 MeV
   range_straggling = model.GetRangeStraggling(10.0); // 10 MeV
   expected = 95.933;                                 // mm
   ASSERT_NEAR(range_straggling, expected, 0.1 * expected);

   model.SetDensity(4e-5);
   range_straggling = model.GetRangeStraggling(1.0); // 1 MeV
   expected = 3.27;                                  // mm
   ASSERT_NEAR(range_straggling, expected, 0.1 * expected);
}

TEST_F(AtELossCATIMATestFixture, TestEnergyLossStraggling)
{

   double expectedSigma = 0.0084 * mass;                                 // Expected sigma from LISE
   double eloss_straggling = model.GetElossStraggling(1.0, 0.75 * mass); // 1 MeV to 10 MeV
   ASSERT_NEAR(eloss_straggling, expectedSigma, 0.1 * expectedSigma);

   expectedSigma = 0.0376 * mass;
   eloss_straggling = model.GetElossStraggling(5.0, 3.58164 * mass);
   ASSERT_NEAR(eloss_straggling, expectedSigma, 0.1 * expectedSigma);
}

TEST_F(AtELossCATIMATestFixture, TestEnergyLossStragglingDistance)
{
   double expectedSigma = 0.0084 * mass;                                  // Expected sigma from LISE
   double eloss_straggling = model.GetElossStragglingDistance(1.0, 50.0); // 1 MeV over 10 mm
   ASSERT_NEAR(eloss_straggling, expectedSigma, 0.1 * expectedSigma);

   expectedSigma = 0.0376 * mass;
   eloss_straggling = model.GetElossStragglingDistance(5.0, 1000.0); // 5 MeV over 100 mm
   ASSERT_NEAR(eloss_straggling, expectedSigma, 0.1 * expectedSigma);
}

TEST_F(AtELossCATIMATestFixture, TestdEdxStraggling)
{
   double dedx = model.GetdEdx(5.0);
   double dE = dedx * 50;
   double expected_dE = model.GetEnergyLoss(5.0, 50.0);

   ASSERT_NEAR(dE, expected_dE, 0.01 * expected_dE); // Verify linear assumption is true

   double E_st = model.GetElossStragglingDistance(5.0, 50.0);
   double dedx_straggling = model.GetdEdxStraggling(5.0, model.GetEnergy(5.0, 50.0));
   ASSERT_NEAR(dedx_straggling * 50, E_st, 0.01 * E_st); // Check dEdx straggling

   double e_min = expected_dE - E_st;
   double e_min_dedx = dE - dedx_straggling * 50;
   ASSERT_NEAR(e_min, e_min_dedx, 0.01 * e_min); // Check minimum energy loss
}

TEST(AtELossCATIMATest, LISE_Match)
{
   // Create the vector with the gas components for H2.
   std::vector<std::tuple<int, int, int>> components;
   components.push_back({1, 1, 2}); // (A, Z, stoichiometry)

   // Calculate the gas density for 600Torr H2 from LISE.
   double density = 6.5643e-5; // g/cm3

   // Create the CATIMA ELoss model.
   AtTools::AtELossCATIMA catimaModel(density, components);

   // Set the projectile to proton.
   catimaModel.SetProjectile(1, 1, 1.007825031898); // (A, Z, massUMA)

   // Calculate ranges for different energies with LISE.
   std::vector<std::pair<double, double>> kinEnergyRange; // (MeV, mm)
   kinEnergyRange.push_back(std::make_pair(1, 1.33e2));
   kinEnergyRange.push_back(std::make_pair(5, 2.2381e3));
   kinEnergyRange.push_back(std::make_pair(10, 7.8883e3));

   // Compare with the results from this class.
   for (auto pair : kinEnergyRange) {
      double energy = pair.first;
      double rangeLISE = pair.second;

      double range = catimaModel.GetRange(energy);
      EXPECT_TRUE(std::abs(range - rangeLISE) / rangeLISE < 0.05);
   }

   // Calculate ELosses for different distances using LISE at 10MeV initial energy.
   std::vector<std::pair<double, double>> distanceELoss; // (mm, MeV)
   distanceELoss.push_back(std::make_pair(100, 0.0694));
   distanceELoss.push_back(std::make_pair(1000, 0.7138));
   distanceELoss.push_back(std::make_pair(5000, 4.2409));

   // Compare with the results from this class.
   for (auto pair : distanceELoss) {
      double distance = pair.first;
      double ELossLISE = pair.second;

      double ELoss = catimaModel.GetEnergyLoss(10, distance);
      EXPECT_TRUE(std::abs(ELoss - ELossLISE) / ELossLISE < 0.05);
   }

   // Do the same for 4He projectile.
   catimaModel.SetProjectile(4, 2, 4.00260325413); // (A, Z, massUMA)

   std::vector<std::pair<double, double>> kinEnergyRange4He; // (MeV, mm)
   kinEnergyRange4He.push_back(std::make_pair(1, 2.5742e1));
   kinEnergyRange4He.push_back(std::make_pair(5, 2.0547e2));
   kinEnergyRange4He.push_back(std::make_pair(10, 6.5724e2));

   // Compare with the results from this class.
   for (auto pair : kinEnergyRange4He) {
      double energy = pair.first;
      double rangeLISE = pair.second;

      double range = catimaModel.GetRange(energy);
      EXPECT_TRUE(std::abs(range - rangeLISE) / rangeLISE < 0.05);
   }

   // Calculate ELosses for different distances using LISE at 10MeV initial energy.
   std::vector<std::pair<double, double>> distanceELoss4He; // (mm, MeV)
   distanceELoss4He.push_back(std::make_pair(100, 0.9121));
   distanceELoss4He.push_back(std::make_pair(500, 5.7726));
   distanceELoss4He.push_back(std::make_pair(1000, 10));

   // Compare with the results from this class.
   for (auto pair : distanceELoss4He) {
      double distance = pair.first;
      double ELossLISE = pair.second;

      double ELoss = catimaModel.GetEnergyLoss(10, distance);
      EXPECT_TRUE(std::abs(ELoss - ELossLISE) / ELossLISE < 0.05);
   }

   // Finally, for 4He, we test if GetRange works when setting a non-zero final energy.
   double energyIni{10};
   double energyFin{4.227};
   double rangeLISE{500};
   double range = catimaModel.GetRange(energyIni, energyFin);
   EXPECT_TRUE(std::abs(range - rangeLISE) / rangeLISE < 0.05);
}
