#include "AtSampleConsensus.h"

#include "AtContainerManip.h"
#include "AtEvent.h" // for AtEvent
#include "AtHit.h"   // for AtHit
#include "AtPattern.h"
#include "AtPatternEvent.h"
#include "AtPatternTypes.h"
#include "AtSample.h" // for AtSample
#include "AtSampleEstimator.h"
#include "AtSampleMethods.h"

#include <FairLogger.h> // for Logger, LOG

#include <fstream> // for std
#include <memory>  // for allocator_traits<>::value_type
#include <set>     // for set, operator!=, _Rb_tree_const_iterator

using namespace SampleConsensus;

AtSampleConsensus::AtSampleConsensus()
   : AtSampleConsensus(Estimators::kRANSAC, PatternType::kLine, SampleMethod::kUniform)
{
}

AtSampleConsensus::AtSampleConsensus(Estimators estimator, PatternType patternType, SampleMethod sampleMethod)
   : fPatternType(patternType), fEstimator(estimator), fRandSampler(RandomSample::CreateSampler(sampleMethod))

{
}

std::unique_ptr<AtPatterns::AtPattern>
AtSampleConsensus::GeneratePatternFromHits(const std::vector<const AtHit *> &hitArray)
{

   if (hitArray.size() < fMinPatternPoints) {
      return nullptr;
   }
   LOG(debug) << "Creating pattern";
   auto pattern = AtPatterns::CreatePattern(fPatternType);
   LOG(debug) << "Sampling points";
   auto points = fRandSampler->SamplePoints(pattern->GetNumPoints());
   LOG(debug) << "Defining pattern";
   pattern->DefinePattern(points);

   LOG(debug) << "Testing pattern";
   auto nInliers = SampleConsensus::AtEstimator::EvaluateModel(pattern.get(), hitArray, fDistanceThreshold, fEstimator);
   LOG(debug) << "Found " << nInliers << " inliers" << std::endl;

   // If the pattern is consistent with enough points, save it
   if (nInliers > fMinPatternPoints) {
      LOG(debug) << "Adding pattern with nInliers: " << nInliers << std::endl;
      return pattern;
   }

   return nullptr;
}

AtPatternEvent AtSampleConsensus::Solve(AtEvent *event)
{
   if (event->IsGood()) {
      auto hitVec = ContainerManip::GetConstPointerVector(event->GetHits());
      return Solve(hitVec);
   }

   return {};
}

AtPatternEvent AtSampleConsensus::Solve(const std::vector<AtHit> &hitArray)
{
   auto hitVec = ContainerManip::GetConstPointerVector(hitArray);
   return Solve(hitVec);
};

AtPatternEvent AtSampleConsensus::Solve(const std::vector<const AtHit *> &hitArray)
{
   if (hitArray.size() < fMinPatternPoints) {
      LOG(error) << "Not enough points to solve. Requires" << fMinPatternPoints;
      return {};
   }
   LOG(info) << "Solving with " << hitArray.size() << " points";

   auto comp = [](const PatternPtr &a, const PatternPtr &b) { return a->GetChi2() < b->GetChi2(); };
   auto sortedPatterns = std::set<PatternPtr, decltype(comp)>(comp);

   LOG(debug2) << "Generating " << fIterations << " patterns";
   fRandSampler->SetHitsToSample(hitArray);
   for (int i = 0; i < fIterations; i++) {
      if (i % 1000 == 0)
         LOG(debug) << "Iteration: " << i << "/" << fIterations;

      auto pattern = GeneratePatternFromHits(hitArray);
      if (pattern != nullptr)
         sortedPatterns.insert(std::move(pattern));
   }
   LOG(debug2) << "Created " << sortedPatterns.size() << " valid patterns.";

   // Loop through each pattern, and extract the points that fit each pattern
   auto remainHits = hitArray;
   AtPatternEvent retEvent;
   for (const auto &pattern : sortedPatterns) {
      if (remainHits.size() < fMinPatternPoints)
         break;

      auto inlierHits = movePointsInPattern(pattern.get(), remainHits);
      if (inlierHits.size() > fMinPatternPoints) {
         auto track = CreateTrack(pattern.get(), inlierHits);
         track.SetTrackID(retEvent.GetTrackCand().size());
         retEvent.AddTrack(track);
      }
   }

   // Add the remaining hits as noise
   for (auto &hit : remainHits)
      retEvent.AddNoise(std::move(*hit));

   return retEvent;
}

AtTrack AtSampleConsensus::CreateTrack(AtPattern *pattern, std::vector<const AtHit *> &inliers)
{
   AtTrack track;

   // Add inliers to our ouput track
   for (auto hit : inliers)
      track.AddHit(std::move(*hit));

   if (fFitPattern)
      pattern->FitPattern(inliers, fChargeThres);

   track.SetPattern(pattern->Clone());
   return track;
}
/**
 * Moves the entries of hits that are consistent with the pattern to the returned vector
 *
 * @param [in] pattern
 * @param[in/out] hits Hits returned are removed from this vector
 * @return vector containing the AtHits consistent with the pattern
 *
 */
std::vector<const AtHit *> AtSampleConsensus::movePointsInPattern(AtPattern *pattern, std::vector<const AtHit *> &hits)
{

   auto isInPattern = [pattern, this](const AtHit *hit) {
      double error = pattern->DistanceToPattern(hit->GetPosition());
      return (error * error) < (fDistanceThreshold * fDistanceThreshold);
   };

   return ContainerManip::MoveFromVector(hits, isInPattern);
}
