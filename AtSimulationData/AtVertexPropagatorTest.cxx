#include "AtVertexPropagator.h"

#include <gtest/gtest.h>

TEST(AtVertexPropagator, IsBeamEvent_BeamEvent)
{
   AtVertexPropagator::Instance()->ResetForTesting();
   AtVertexPropagator::Instance()->SetIsBeamEvent(true);

   EXPECT_TRUE(AtVertexPropagator::Instance()->IsBeamEvent());
   EXPECT_FALSE(AtVertexPropagator::Instance()->IsReactionEvent());
}

TEST(AtVertexPropagator, IsBeamEvent_ReactionEvent)
{
   AtVertexPropagator::Instance()->ResetForTesting();
   AtVertexPropagator::Instance()->SetIsBeamEvent(false);

   EXPECT_TRUE(AtVertexPropagator::Instance()->IsReactionEvent());
   EXPECT_FALSE(AtVertexPropagator::Instance()->IsBeamEvent());
}

TEST(AtVertexPropagator, EndEvent)
{
   AtVertexPropagator::Instance()->ResetForTesting();
   AtVertexPropagator::Instance()->SetIsBeamEvent(true);
   AtVertexPropagator::Instance()->EndEvent();

   EXPECT_TRUE(AtVertexPropagator::Instance()->IsReactionEvent());
   EXPECT_FALSE(AtVertexPropagator::Instance()->IsBeamEvent());
   AtVertexPropagator::Instance()->EndEvent();
   EXPECT_TRUE(AtVertexPropagator::Instance()->IsBeamEvent());
   EXPECT_FALSE(AtVertexPropagator::Instance()->IsReactionEvent());
}

TEST(AtVertexPropagator, SetVertexStoresBeamState)
{
   AtVertexPropagator::Instance()->ResetForTesting();
   AtVertexPropagator::Instance()->SetVertex(1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 0.1, 0.2, 0.3, 12.5);

   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVx(), 1.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVy(), 2.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVz(), 3.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetInVx(), -1.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetInVy(), -2.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetInVz(), -3.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetPx(), 0.1);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetPy(), 0.2);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetPz(), 0.3);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetEnergy(), 12.5);
}

TEST(AtVertexPropagator, ResetVertexClearsVertexAndTrackMetadata)
{
   AtVertexPropagator::Instance()->ResetForTesting();
   AtVertexPropagator::Instance()->SetVertex(1.0, 2.0, 3.0, -1.0, -2.0, -3.0, 0.1, 0.2, 0.3, 12.5);
   AtVertexPropagator::Instance()->SetTrackEnergy(1, 5.0);
   AtVertexPropagator::Instance()->SetTrackAngle(1, 45.0);

   AtVertexPropagator::Instance()->ResetVertex();

   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVx(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVy(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVz(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetPx(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetPy(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetPz(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetEnergy(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetTrackEnergy(1), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetTrackAngle(1), 0.0);
}
