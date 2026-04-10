#include "AtELossModelFactory.h"

#include "AtELossFactoryBetheBloch.h"
#include "AtELossModel.h"

#include <TGeoManager.h>
#include <TGeoMaterial.h>

#include <cmath>
#include <gtest/gtest.h>
#include <tuple>
#include <vector>

using namespace AtTools;

// TGeoMaterial/TGeoMixture constructors require gGeoManager to exist.
// Materials are heap-allocated so TGeoManager can own and clean them up.
class GeoFixture : public ::testing::Test {
protected:
   static void SetUpTestSuite()
   {
      if (gGeoManager == nullptr)
         new TGeoManager("test", "test geometry");
   }
};

// ---- WeightFractionsToStoichiometry tests ----

TEST(AtELossModelFactoryUtils, WaterStoichiometry)
{
   // H2O: H weight fraction ~0.1119, O ~0.8881
   std::vector<double> weights = {0.111898, 0.888102};
   std::vector<double> masses = {1.008, 15.999};
   auto stoich = AtELossModelFactory::WeightFractionsToStoichiometry(weights, masses);

   ASSERT_EQ(stoich.size(), 2u);
   EXPECT_EQ(stoich[0], 2); // H
   EXPECT_EQ(stoich[1], 1); // O
}

TEST(AtELossModelFactoryUtils, CO2Stoichiometry)
{
   // CO2: C ~0.2729, O ~0.7271
   std::vector<double> weights = {0.272916, 0.727084};
   std::vector<double> masses = {12.011, 15.999};
   auto stoich = AtELossModelFactory::WeightFractionsToStoichiometry(weights, masses);

   ASSERT_EQ(stoich.size(), 2u);
   EXPECT_EQ(stoich[0], 1); // C
   EXPECT_EQ(stoich[1], 2); // O
}

TEST(AtELossModelFactoryUtils, PureElementStoichiometry)
{
   std::vector<double> weights = {1.0};
   std::vector<double> masses = {4.003};
   auto stoich = AtELossModelFactory::WeightFractionsToStoichiometry(weights, masses);

   ASSERT_EQ(stoich.size(), 1u);
   EXPECT_EQ(stoich[0], 1);
}

TEST(AtELossModelFactoryUtils, EmptyInput)
{
   auto stoich = AtELossModelFactory::WeightFractionsToStoichiometry({}, {});
   EXPECT_TRUE(stoich.empty());
}

// ---- ExtractComposition tests (heap-allocated materials for TGeoManager ownership) ----

TEST_F(GeoFixture, ExtractPureMaterial)
{
   auto *mat = new TGeoMaterial("He_extract", 4.003, 2, 1.664e-4);
   auto comp = AtELossModelFactory::ExtractComposition(mat);

   ASSERT_EQ(comp.size(), 1u);
   auto [A, Z, s] = comp[0];
   EXPECT_EQ(A, 4);
   EXPECT_EQ(Z, 2);
   EXPECT_EQ(s, 1);
}

TEST_F(GeoFixture, ExtractMixture)
{
   // AddElement signature: AddElement(A, Z, weight)
   auto *mix = new TGeoMixture("HeCO2_extract", 3, 1.0e-3);
   mix->AddElement(4, 2, 0.90);  // He
   mix->AddElement(12, 6, 0.03); // C
   mix->AddElement(16, 8, 0.07); // O

   auto comp = AtELossModelFactory::ExtractComposition(mix);
   ASSERT_EQ(comp.size(), 3u);

   EXPECT_EQ(std::get<1>(comp[0]), 2); // He Z
   EXPECT_EQ(std::get<1>(comp[1]), 6); // C Z
   EXPECT_EQ(std::get<1>(comp[2]), 8); // O Z

   for (const auto &[a, z, s] : comp)
      EXPECT_GE(s, 1);
}

TEST(AtELossModelFactoryUtils, ExtractNullMaterial)
{
   auto comp = AtELossModelFactory::ExtractComposition(nullptr);
   EXPECT_TRUE(comp.empty());
}

// ---- EffectiveMeanIonization tests ----

TEST_F(GeoFixture, PureMaterialIonization)
{
   auto *mat = new TGeoMaterial("H_ionize", 1.008, 1, 8.376e-5);
   double I = AtELossModelFactory::EffectiveMeanIonization(mat);
   EXPECT_DOUBLE_EQ(I, 13.5); // 13.5 * Z=1
}

TEST_F(GeoFixture, PureHeliumIonization)
{
   auto *mat = new TGeoMaterial("He_ionize", 4.003, 2, 1.664e-4);
   double I = AtELossModelFactory::EffectiveMeanIonization(mat);
   EXPECT_DOUBLE_EQ(I, 27.0); // 13.5 * Z=2
}

TEST_F(GeoFixture, MixtureIonization)
{
   // Pure hydrogen mixture: should give I = 13.5 eV
   auto *mix = new TGeoMixture("H2_ionize", 1, 8.376e-5);
   mix->AddElement(1, 1, 1.0);
   double I = AtELossModelFactory::EffectiveMeanIonization(mix);
   EXPECT_NEAR(I, 13.5, 0.5);
}

TEST(AtELossModelFactoryUtils, NullIonization)
{
   double I = AtELossModelFactory::EffectiveMeanIonization(nullptr);
   EXPECT_DOUBLE_EQ(I, 0.0);
}

// ---- BetheBloch factory CreateModel test ----

TEST_F(GeoFixture, CreateFromPureMaterial)
{
   auto *mat = new TGeoMaterial("H_bbfactory", 1.008, 1, 6.5643e-5);
   AtELossFactoryBetheBloch factory;

   auto model = factory.CreateModel(1, 1, 1.007825, mat);
   ASSERT_NE(model, nullptr);

   double dedx = model->GetdEdx(10.0);
   EXPECT_GT(dedx, 0.0);

   double range = model->GetRange(10.0);
   EXPECT_GT(range, 0.0);
   EXPECT_LT(range, 1e8);
}

TEST_F(GeoFixture, CreateFromMixture)
{
   auto *mix = new TGeoMixture("HeCO2_bbfactory", 3, 1.0e-3);
   mix->AddElement(4, 2, 0.90);
   mix->AddElement(12, 6, 0.03);
   mix->AddElement(16, 8, 0.07);

   AtELossFactoryBetheBloch factory;
   auto model = factory.CreateModel(2, 4, 4.002603, mix);
   ASSERT_NE(model, nullptr);

   double dedx = model->GetdEdx(10.0);
   EXPECT_GT(dedx, 0.0);
}

TEST_F(GeoFixture, NullMaterial)
{
   AtELossFactoryBetheBloch factory;
   auto model = factory.CreateModel(1, 1, 1.007825, nullptr);
   EXPECT_EQ(model, nullptr);
}
