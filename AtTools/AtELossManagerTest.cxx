#include "AtELossManager.h"

#include "AtELossBetheBloch.h"
#include "AtELossManagerBetheBloch.h"
#include "AtELossModel.h"

#include <TGeoManager.h>
#include <TGeoMaterial.h>

#include <cmath>
#include <gtest/gtest.h>
#include <memory>
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

TEST(AtELossManagerUtils, WaterStoichiometry)
{
   std::vector<double> weights = {0.111898, 0.888102};
   std::vector<double> masses = {1.008, 15.999};
   auto stoich = AtELossManager::WeightFractionsToStoichiometry(weights, masses);

   ASSERT_EQ(stoich.size(), 2u);
   EXPECT_EQ(stoich[0], 2); // H
   EXPECT_EQ(stoich[1], 1); // O
}

TEST(AtELossManagerUtils, CO2Stoichiometry)
{
   std::vector<double> weights = {0.272916, 0.727084};
   std::vector<double> masses = {12.011, 15.999};
   auto stoich = AtELossManager::WeightFractionsToStoichiometry(weights, masses);

   ASSERT_EQ(stoich.size(), 2u);
   EXPECT_EQ(stoich[0], 1); // C
   EXPECT_EQ(stoich[1], 2); // O
}

TEST(AtELossManagerUtils, PureElementStoichiometry)
{
   std::vector<double> weights = {1.0};
   std::vector<double> masses = {4.003};
   auto stoich = AtELossManager::WeightFractionsToStoichiometry(weights, masses);

   ASSERT_EQ(stoich.size(), 1u);
   EXPECT_EQ(stoich[0], 1);
}

TEST(AtELossManagerUtils, EmptyInput)
{
   auto stoich = AtELossManager::WeightFractionsToStoichiometry({}, {});
   EXPECT_TRUE(stoich.empty());
}

// ---- ExtractComposition tests ----

TEST_F(GeoFixture, ExtractPureMaterial)
{
   auto *mat = new TGeoMaterial("He_extract", 4.003, 2, 1.664e-4);
   auto comp = AtELossManager::ExtractComposition(mat);

   ASSERT_EQ(comp.size(), 1u);
   auto [A, Z, s] = comp[0];
   EXPECT_EQ(A, 4);
   EXPECT_EQ(Z, 2);
   EXPECT_EQ(s, 1);
}

TEST_F(GeoFixture, ExtractMixture)
{
   auto *mix = new TGeoMixture("HeCO2_extract", 3, 1.0e-3);
   mix->AddElement(4, 2, 0.90);
   mix->AddElement(12, 6, 0.03);
   mix->AddElement(16, 8, 0.07);

   auto comp = AtELossManager::ExtractComposition(mix);
   ASSERT_EQ(comp.size(), 3u);

   EXPECT_EQ(std::get<1>(comp[0]), 2); // He Z
   EXPECT_EQ(std::get<1>(comp[1]), 6); // C Z
   EXPECT_EQ(std::get<1>(comp[2]), 8); // O Z

   for (const auto &[a, z, s] : comp)
      EXPECT_GE(s, 1);
}

TEST(AtELossManagerUtils, ExtractNullMaterial)
{
   auto comp = AtELossManager::ExtractComposition(nullptr);
   EXPECT_TRUE(comp.empty());
}

// ---- EffectiveMeanIonization tests ----

TEST_F(GeoFixture, PureMaterialIonization)
{
   auto *mat = new TGeoMaterial("H_ionize", 1.008, 1, 8.376e-5);
   double I = AtELossManager::EffectiveMeanIonization(mat);
   EXPECT_DOUBLE_EQ(I, 13.5);
}

TEST_F(GeoFixture, PureHeliumIonization)
{
   auto *mat = new TGeoMaterial("He_ionize", 4.003, 2, 1.664e-4);
   double I = AtELossManager::EffectiveMeanIonization(mat);
   EXPECT_DOUBLE_EQ(I, 27.0);
}

TEST_F(GeoFixture, MixtureIonization)
{
   auto *mix = new TGeoMixture("H2_ionize", 1, 8.376e-5);
   mix->AddElement(1, 1, 1.0);
   double I = AtELossManager::EffectiveMeanIonization(mix);
   EXPECT_NEAR(I, 13.5, 0.5);
}

TEST(AtELossManagerUtils, NullIonization)
{
   double I = AtELossManager::EffectiveMeanIonization(nullptr);
   EXPECT_DOUBLE_EQ(I, 0.0);
}

// ---- BetheBloch manager (autogenerate) tests ----

TEST_F(GeoFixture, BBGenerateFromPureMaterial)
{
   auto *mat = new TGeoMaterial("H_bbgen", 1.008, 1, 6.5643e-5);
   AtELossManagerBetheBloch manager;

   auto model = manager.GetModel(1, 1, 1.007825, mat);
   ASSERT_NE(model, nullptr);
   EXPECT_GT(model->GetdEdx(10.0), 0.0);
}

TEST_F(GeoFixture, BBGenerateFromMixture)
{
   auto *mix = new TGeoMixture("HeCO2_bbgen", 3, 1.0e-3);
   mix->AddElement(4, 2, 0.90);
   mix->AddElement(12, 6, 0.03);
   mix->AddElement(16, 8, 0.07);

   AtELossManagerBetheBloch manager;
   auto model = manager.GetModel(2, 4, 4.002603, mix);
   ASSERT_NE(model, nullptr);
   EXPECT_GT(model->GetdEdx(10.0), 0.0);
}

TEST_F(GeoFixture, GenerateNullMaterialReturnsNull)
{
   AtELossManagerBetheBloch manager;
   auto model = manager.GetModel(1, 1, 1.007825, nullptr);
   EXPECT_EQ(model, nullptr);
}

// ---- Registration and priority tests ----

namespace {
// Minimal stand-in AtELossModel: fixed values so tests can detect which instance was served.
class StubModel : public AtELossModel {
public:
   explicit StubModel(double tag) : AtELossModel(1), fTag(tag) {}
   double GetdEdx(double /*e*/) const override { return fTag; }
   double GetRange(double /*ei*/, double /*ef*/ = 0) const override { return 0; }
   double GetEnergyLoss(double /*ei*/, double /*d*/) const override { return 0; }
   double GetEnergy(double /*ei*/, double /*d*/) const override { return 0; }
   double GetElossStraggling(double /*ei*/, double /*ef*/) const override { return 0; }
   double GetdEdxStraggling(double /*ei*/, double /*ef*/) const override { return 0; }
   double GetRangeVariance(double /*e*/) const override { return 0; }

private:
   double fTag;
};
} // namespace

TEST_F(GeoFixture, MaterialAgnosticRegistrationServedRegardlessOfMaterial)
{
   auto *mat = new TGeoMaterial("M_agnostic", 1.008, 1, 6.5643e-5);
   AtELossManager manager; // accept-only base class
   manager.AddModel(1, 1, std::make_shared<StubModel>(42.0));

   auto model = manager.GetModel(1, 1, 1.007825, mat);
   ASSERT_NE(model, nullptr);
   EXPECT_DOUBLE_EQ(model->GetdEdx(1.0), 42.0);
}

TEST_F(GeoFixture, MaterialSpecificRegistrationTakesPriority)
{
   auto *mat = new TGeoMaterial("M_priority", 1.008, 1, 6.5643e-5);
   AtELossManager manager;
   manager.AddModel(1, 1, std::make_shared<StubModel>(1.0));
   manager.AddModel(1, 1, "M_priority", std::make_shared<StubModel>(2.0));

   auto model = manager.GetModel(1, 1, 1.007825, mat);
   ASSERT_NE(model, nullptr);
   EXPECT_DOUBLE_EQ(model->GetdEdx(1.0), 2.0); // material-specific wins
}

TEST_F(GeoFixture, SameZaDifferentMaterialsGiveDifferentModels)
{
   auto *mat1 = new TGeoMaterial("M_first", 4.003, 2, 1.664e-4);
   auto *mat2 = new TGeoMaterial("M_second", 1.008, 1, 6.5643e-5);
   AtELossManagerBetheBloch manager;

   auto m1 = manager.GetModel(1, 1, 1.007825, mat1);
   auto m2 = manager.GetModel(1, 1, 1.007825, mat2);
   ASSERT_NE(m1, nullptr);
   ASSERT_NE(m2, nullptr);
   EXPECT_NE(m1.get(), m2.get());
}

TEST_F(GeoFixture, CacheReturnsSameInstanceOnRepeatedCalls)
{
   auto *mat = new TGeoMaterial("M_cache", 1.008, 1, 6.5643e-5);
   AtELossManagerBetheBloch manager;

   auto m1 = manager.GetModel(1, 1, 1.007825, mat);
   auto m2 = manager.GetModel(1, 1, 1.007825, mat);
   ASSERT_NE(m1, nullptr);
   EXPECT_EQ(m1.get(), m2.get());
}

TEST_F(GeoFixture, ClearCacheDropsGeneratedButKeepsRegistered)
{
   auto *mat = new TGeoMaterial("M_clear", 1.008, 1, 6.5643e-5);
   AtELossManagerBetheBloch manager;
   manager.AddModel(2, 4, std::make_shared<StubModel>(7.0));

   auto generated = manager.GetModel(1, 1, 1.007825, mat);
   ASSERT_NE(generated, nullptr);

   manager.ClearCache();

   // Registered model survives the ClearCache and can still be served.
   auto registered = manager.GetModel(2, 4, 4.002603, mat);
   ASSERT_NE(registered, nullptr);
   EXPECT_DOUBLE_EQ(registered->GetdEdx(1.0), 7.0);

   // Generated model is re-created on next query (fresh pointer after cache clear).
   auto regenerated = manager.GetModel(1, 1, 1.007825, mat);
   ASSERT_NE(regenerated, nullptr);
   EXPECT_NE(regenerated.get(), generated.get());
}

TEST_F(GeoFixture, BaseManagerDoesNotGenerate)
{
   auto *mat = new TGeoMaterial("M_nogen", 1.008, 1, 6.5643e-5);
   AtELossManager manager; // no GenerateModel override
   auto model = manager.GetModel(1, 1, 1.007825, mat);
   EXPECT_EQ(model, nullptr);
}
