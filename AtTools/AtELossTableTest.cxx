#include "AtELossTable.h"

#include <FairLogger.h>

#include <cmath>
#include <gtest/gtest.h>
using namespace AtTools;

/**
 * Helper function to get the path to the energy loss table
 */
std::string getEnergyPath()
{
   auto env = std::getenv("VMCWORKDIR");
   if (env == nullptr) {
      return "../../resources/energy_loss/HinH.txt"; // Default path assuming cwd is build/AtTools
   }
   return std::string(env) + "/resources/energy_loss/HinH.txt"; // Use environment variable
}

/**
 * Test fixture for the AtELossTable class that initializes a H2 gas w/ proton model.
 */
class AtELossTableTestFixture : public ::testing::Test {
protected:
   AtELossTable tableModel;
   double mass{1.007825031898}; // Mass of proton in amu

   AtELossTableTestFixture() : tableModel(0)
   {
      // Initialize the table model with H2 gas density and load SRIM table for proton.
      tableModel.LoadSrimTable(getEnergyPath());
      tableModel.SetDensity(6.5643e-5);
   }
};

TEST_F(AtELossTableTestFixture, ConstructTableModel)
{
   // Basic check: ensure model is constructed and can compute range
   double range = tableModel.GetRange(1.0); // 1 MeV
   EXPECT_GT(range, 0.0);
}

TEST_F(AtELossTableTestFixture, TestRange)
{
   tableModel.SetDensity(4.1906E-05);
   ASSERT_NEAR(tableModel.GetRange(1.0), 202.78, 0.05);
   ASSERT_NEAR(tableModel.GetRange(5.0), 3580, 10);
}

TEST_F(AtELossTableTestFixture, TestRangeStraggling)
{
   // Check range straggling for known energies
   // tableModel.SetDensity(4.1906E-05);

   double range_var = tableModel.GetRangeVariance(1.0); // 1 MeV
   double expected = 5.58;                              // Expected value from SRIM table (LISE is different)
   ASSERT_NEAR(range_var, expected * expected, 0.2 * expected * expected);
   double range_straggling = tableModel.GetRangeStraggling(1.0); // 1 MeV
   ASSERT_NEAR(range_straggling, expected, 0.1 * expected);

   // Check straggling at 10 MeV
   range_straggling = tableModel.GetRangeStraggling(10.0); // 10 MeV
   expected = 362.26;                                      // mm
   ASSERT_NEAR(range_straggling, expected, 0.1 * expected);
}

TEST_F(AtELossTableTestFixture, TestEnergyLossStraggling)
{
   double expectedSigma = .67 * 6.56 * 1e-3 * std::sqrt(5.58 * 5.58 - 2.67 * 2.67); // Expected sigma from LISE
   double eloss_straggling = tableModel.GetElossStraggling(1.0, 0.7);               // 1 MeV to 0.7 MeV
   ASSERT_NEAR(eloss_straggling, expectedSigma, 0.1 * expectedSigma);
}

TEST_F(AtELossTableTestFixture, DebugEnergyLossCalculation)
{
   double E0 = 5.0;
   double distance = 10.0;
   // tableModel.SetDensity(4.1906E-05);

   // Check dE/dx variation over the path
   double dedx_start = tableModel.GetdEdx(E0);
   double E_end = tableModel.GetEnergy(E0, distance);
   double dedx_end = tableModel.GetdEdx(E_end);

   LOG(info) << "E0: " << E0 << " MeV, E_end: " << E_end << " MeV";
   LOG(info) << "dE/dx at start: " << dedx_start << " MeV/mm";
   LOG(info) << "dE/dx at end: " << dedx_end << " MeV/mm";
   LOG(info) << "dE/dx variation: " << (dedx_end - dedx_start) / dedx_start * 100 << "%";

   // Compare calculations
   double linear_loss = dedx_start * distance;
   double integrated_loss = tableModel.GetEnergyLoss(E0, distance);
   double energy_diff_loss = E0 - E_end;

   LOG(info) << "Linear approximation: " << linear_loss << " MeV";
   LOG(info) << "Integrated energy loss: " << integrated_loss << " MeV";
   LOG(info) << "Energy difference: " << energy_diff_loss << " MeV";

   LOG(info) << "Linear vs Integrated error: " << (linear_loss - integrated_loss) / integrated_loss * 100 << "%";
   LOG(info) << "Integrated vs Energy diff error: " << (integrated_loss - energy_diff_loss) / energy_diff_loss * 100
             << "%";

   // These should all be very close if dE/dx only varies by 0.1%
   EXPECT_NEAR(linear_loss, integrated_loss, 0.02 * integrated_loss);
   EXPECT_NEAR(integrated_loss, energy_diff_loss, 0.01 * energy_diff_loss);
}

TEST_F(AtELossTableTestFixture, TestdEdxStraggling)
{
   double dedx = tableModel.GetdEdx(5.0);
   double dE = dedx * 10;
   double expected_dE = tableModel.GetEnergyLoss(5.0, 10.0);

   ASSERT_NEAR(dE, expected_dE, 0.1 * expected_dE); // Verify linear assumption is true

   double E_st = tableModel.GetElossStraggling(5.0, tableModel.GetEnergy(5.0, 10.0));
   double dedx_straggling = tableModel.GetdEdxStraggling(5.0, tableModel.GetEnergy(5.0, 10.0));
   ASSERT_NEAR(dedx_straggling * 10, E_st, 0.01 * E_st); // Check dEdx straggling

   double e_min = expected_dE - E_st;
   double e_min_dedx = dE - dedx_straggling * 10;
   ASSERT_NEAR(e_min, e_min_dedx, 0.02 * e_min); // Check minimum energy loss
}
