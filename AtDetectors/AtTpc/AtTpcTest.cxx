#include "AtTpc.h"

#include "AtMCPoint.h"
#include "AtVertexPropagator.h"

#include <TClonesArray.h>

#include <gtest/gtest.h>

namespace {
AtTpc::StepState MakeStep(int trackID, int pdg, const char *volumeName, double eLossGeV, double totalEnergyGeV, double zCm)
{
   AtTpc::StepState step;
   step.trackID = trackID;
   step.pdg = pdg;
   step.volumeName = volumeName;
   step.volumeID = 1;
   step.detCopyID = 0;
   step.beamTrack = false;
   step.energyLoss = eLossGeV;
   step.trackLength = zCm;
   step.totalEnergy = totalEnergyGeV;
   step.trackMass = 0.938272;
   step.pos.SetXYZT(0.0, 0.0, zCm, 0.0);
   step.mom.SetXYZT(0.0, 0.0, 0.04, totalEnergyGeV);
   step.posOut = step.pos;
   step.momOut = step.mom;
   return step;
}
} // namespace

class AtTpcTest : public ::testing::Test {
protected:
   AtTpc detector;

   void SetUp() override
   {
      AtVertexPropagator::Instance()->ResetForTesting();
      AtVertexPropagator::Instance()->SetBeamMass(1.007276);
      AtVertexPropagator::Instance()->ResetVertex();
      detector.Reset();
   }

   void TearDown() override { AtVertexPropagator::Instance()->ResetForTesting(); }
};

TEST_F(AtTpcTest, ReactionTriggerPopulatesVertexPropagator)
{
   AtVertexPropagator::Instance()->SetRndELoss(0.5);

   auto step = MakeStep(0, 2212, "drift_volume", 0.0006, 0.98, 12.0);
   step.beamTrack = true;
   step.entering = true;

   const bool stopTransport = detector.ProcessStep(step);

   EXPECT_TRUE(stopTransport);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVx(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVy(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVz(), 12.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetInVz(), 12.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetPz(), 0.04);

   const double expectedEnergy = (0.98 - 1.007276 * 0.93149401) * 1000.0;
   EXPECT_NEAR(AtVertexPropagator::Instance()->GetEnergy(), expectedEnergy, 1e-6);
}

TEST_F(AtTpcTest, BeamExitResetsVertexState)
{
   AtVertexPropagator::Instance()->SetVertex(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 100.0);

   auto step = MakeStep(0, 2212, "drift_volume", 0.0, 0.95, 20.0);
   step.beamTrack = true;
   step.exiting = true;
   step.posOut.SetXYZT(0.0, 0.0, 25.0, 0.0);

   detector.ProcessStep(step);

   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVz(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetEnergy(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetPz(), 0.0);
}

TEST_F(AtTpcTest, ReactionEventTrackZeroDoesNotTriggerBeamHandoff)
{
   AtVertexPropagator::Instance()->SetIsBeamEvent(false);
   AtVertexPropagator::Instance()->SetRndELoss(0.5);

   auto step = MakeStep(0, 1000060160, "drift_volume", 0.0006, 15.9, 12.0);
   step.entering = true;

   const bool stopTransport = detector.ProcessStep(step);

   EXPECT_FALSE(stopTransport);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVz(), 0.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetEnergy(), 0.0);
}

TEST_F(AtTpcTest, ReactionEventTrackZeroExitDoesNotResetVertexState)
{
   AtVertexPropagator::Instance()->SetIsBeamEvent(false);
   AtVertexPropagator::Instance()->SetVertex(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 100.0);

   auto step = MakeStep(0, 1000060160, "drift_volume", 0.0, 15.9, 20.0);
   step.exiting = true;
   step.posOut.SetXYZT(0.0, 0.0, 25.0, 0.0);

   detector.ProcessStep(step);

   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetVz(), 3.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetEnergy(), 100.0);
   EXPECT_DOUBLE_EQ(AtVertexPropagator::Instance()->GetPz(), 1.0);
}

TEST_F(AtTpcTest, ExitingReactionVolumeStopsTransportWhenFlagSet)
{
   detector.SetStopOnReactionVolumeExit(true);
   auto step = MakeStep(1, 2212, "drift_volume", 0.0, 0.95, 20.0);
   step.exiting = true;
   step.posOut.SetXYZT(0.0, 0.0, 25.0, 0.0);

   const bool stopTransport = detector.ProcessStep(step);

   EXPECT_TRUE(stopTransport);
}

TEST_F(AtTpcTest, ExitingReactionVolumeDoesNotStopByDefault)
{
   auto step = MakeStep(1, 2212, "drift_volume", 0.0, 0.95, 20.0);
   step.exiting = true;
   step.posOut.SetXYZT(0.0, 0.0, 25.0, 0.0);

   const bool stopTransport = detector.ProcessStep(step);

   EXPECT_FALSE(stopTransport);
}

TEST_F(AtTpcTest, NonBeamTracksUseStoredMetadata)
{
   AtVertexPropagator::Instance()->SetTrackEnergy(1, 7.5);
   AtVertexPropagator::Instance()->SetTrackAngle(1, 32.0);

   auto step = MakeStep(1, 1000020040, "drift_volume", 0.0001, 3.8, 8.0);
   detector.ProcessStep(step);

   auto *points = detector.GetCollection(0);
   ASSERT_NE(points, nullptr);
   ASSERT_EQ(points->GetEntriesFast(), 1);

   auto *point = dynamic_cast<AtMCPoint *>(points->At(0));
   ASSERT_NE(point, nullptr);
   EXPECT_DOUBLE_EQ(point->GetEIni(), 7.5);
   EXPECT_DOUBLE_EQ(point->GetAIni(), 32.0);
   EXPECT_EQ(point->GetMassNum(), 4);
   EXPECT_EQ(point->GetAtomicNum(), 2);
}
