#include "AtkNN.h"

#include "AtHit.h"

#include <gtest/gtest.h>
using XYZPoint = ROOT::Math::XYZPoint;

TEST(AtkNNTest, AllOneAway)
{

   // Create a sample HitCloud
   AtTools::DataCleaning::HitCloud hits;
   hits.push_back(std::make_unique<AtHit>(-1, XYZPoint(0, 0, 0), 0));
   hits.push_back(std::make_unique<AtHit>(-1, XYZPoint(1, 0, 0), 0));
   hits.push_back(std::make_unique<AtHit>(-1, XYZPoint(0, 1, 0), 0));

   // Create an instance of AtkNN
   AtTools::DataCleaning::AtkNN atkNN(1, 0.5);
   AtTools::DataCleaning::HitCloud cleanedHits = atkNN.CleanData(hits);
   EXPECT_EQ(0, cleanedHits.size());

   atkNN = AtTools::DataCleaning::AtkNN(1, 1.1);
   cleanedHits = atkNN.CleanData(hits);
   EXPECT_EQ(3, cleanedHits.size());

   atkNN = AtTools::DataCleaning::AtkNN(2, 1.1);
   cleanedHits = atkNN.CleanData(hits);
   EXPECT_EQ(1, cleanedHits.size());

   atkNN = AtTools::DataCleaning::AtkNN(2, 1.5);
   cleanedHits = atkNN.CleanData(hits);
   EXPECT_EQ(3, cleanedHits.size());
}