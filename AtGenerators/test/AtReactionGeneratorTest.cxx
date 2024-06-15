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

TEST(AtReactionGenerator, AtReactionGenerator_ReadEvent_BeamEvent)
{
   FakeAtReactionGenerator reactionImp;
   AtVertexPropagator::Instance()->ResetForTesting();
   AtVertexPropagator::Instance()->SetIsBeamEvent(true);
   reactionImp.ReadEvent(nullptr);
   EXPECT_FALSE(reactionImp.RanEvent());
}

TEST(AtReactionGenerator, AtReactionGenerator_ReadEvent_NonBeamEvent)
{
   FakeAtReactionGenerator reactionImp;
   AtVertexPropagator::Instance()->ResetForTesting();
   AtVertexPropagator::Instance()->SetIsBeamEvent(false);
   reactionImp.ReadEvent(nullptr);
   EXPECT_TRUE(reactionImp.RanEvent());
}