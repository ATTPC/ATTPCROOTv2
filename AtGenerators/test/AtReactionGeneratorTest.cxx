#include "AtReactionGenerator.h"

#include "AtVertexPropagator.h"

#include <gtest/gtest.h>

class FakeAtReactionGenerator : public AtReactionGenerator {
protected:
   bool ranEvent{false};

public:
   bool GenerateReaction(FairPrimaryGenerator *primGen) override
   {
      ranEvent = true;
      return true;
   }
   bool RanEvent() { return ranEvent; }
};

class AtReactionGeneratorTest : public ::testing::Test {
protected:
   FakeAtReactionGenerator generator;
   void SetUp() override { AtVertexPropagator::Instance()->ResetForTesting(); }
   void TearDown() override { AtVertexPropagator::Instance()->ResetForTesting(); }
};

TEST_F(AtReactionGeneratorTest, AtReactionGenerator_ReadEvent_BeamEvent)
{

   AtVertexPropagator::Instance()->ResetForTesting();
   AtVertexPropagator::Instance()->SetIsBeamEvent(true);
   generator.ReadEvent(nullptr);
   EXPECT_FALSE(generator.RanEvent());
}

TEST_F(AtReactionGeneratorTest, AtReactionGenerator_ReadEvent_NonBeamEvent)
{

   AtVertexPropagator::Instance()->SetIsBeamEvent(false);
   generator.ReadEvent(nullptr);
   EXPECT_TRUE(generator.RanEvent());
}

TEST_F(AtReactionGeneratorTest, AtReactionGenerator_ReadEvent_AlwaysNonBeamEvent)
{

   AtVertexPropagator::Instance()->SetIsBeamEvent(true);
   generator.SetAlwaysRun(true);
   generator.ReadEvent(nullptr);
   EXPECT_TRUE(generator.RanEvent());
}