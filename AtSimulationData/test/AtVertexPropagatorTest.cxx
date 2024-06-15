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