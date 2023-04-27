#include "AtEstimatorMethods.h"

#include "AtContainerManip.h"
#include "AtHit.h" // for AtHit
#include "AtPattern.h"
#include "AtPatternY.h"

#include <algorithm> // for max_element, nth_element, max
#include <cassert>
#include <cmath> // for exp, sqrt, isinf, log, M_PI
using namespace SampleConsensus;

int SampleConsensus::EvaluateChi2(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray,
                                  double distanceThreshold)
{
   int nbInliers = 0;
   double weight = 0;

   for (const auto &hit : hitArray) {
      auto &pos = hit->GetPosition();
      double error = model->DistanceToPattern(pos);
      error = error * error;
      if (error < (distanceThreshold * distanceThreshold)) {
         nbInliers++;
         weight += error;
      }
   }
   model->SetChi2(weight / nbInliers);
   return nbInliers;
}
int SampleConsensus::EvaluateRansac(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray,
                                    double distanceThreshold)
{
   int nbInliers = 0;
   for (const auto &hit : hitArray) {
      auto &pos = hit->GetPosition();
      double error = model->DistanceToPattern(pos);
      error = error * error;
      if (error < (distanceThreshold * distanceThreshold)) {
         nbInliers++;
      }
   }
   model->SetChi2(1.0 / nbInliers);
   return nbInliers;
}

int SampleConsensus::EvaluateYRansac(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray,
                                     double distanceThreshold)
{
   auto *yModel = dynamic_cast<AtPatterns::AtPatternY *>(model);
   assert(yModel != nullptr);

   int nbInliers = 0;
   for (const auto &hit : hitArray) {
      auto &pos = hit->GetPosition();
      if (yModel->GetPointAssignment(pos) >= 2)
         continue;
      double error = model->DistanceToPattern(pos);
      error = error * error;
      if (error < (distanceThreshold * distanceThreshold)) {
         nbInliers++;
      }
   }
   model->SetChi2(1.0 / nbInliers);
   return nbInliers;
}

int SampleConsensus::EvaluateMlesac(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray,
                                    double distanceThreshold)
{
   double sigma = distanceThreshold / 1.96;
   double dataSigma2 = sigma * sigma;

   // Calculate min and max errors
   double minError = 1e5, maxError = -1e5;
   for (const auto &hit : hitArray) {
      double error = model->DistanceToPattern(hit->GetPosition());
      if (error < minError)
         minError = error;
      if (error > maxError)
         maxError = error;
   }

   // Estimate the inlier ratio using EM
   const double nu = maxError - minError;
   double gamma = 0.5;
   for (int iter = 0; iter < 5; iter++) {
      double sumPosteriorProb = 0;
      const double probOutlier = (1 - gamma) / nu;
      const double probInlierCoeff = gamma / sqrt(2 * M_PI * dataSigma2);

      for (const auto &hit : hitArray) {
         double error = model->DistanceToPattern(hit->GetPosition());
         double probInlier = probInlierCoeff * exp(-0.5 * error * error / dataSigma2);
         sumPosteriorProb += probInlier / (probInlier + probOutlier);
      }
      gamma = sumPosteriorProb / hitArray.size();
   }

   double sumLogLikelihood = 0;
   int nbInliers = 0;

   // Evaluate the model
   const double probOutlier = (1 - gamma) / nu;
   const double probInlierCoeff = gamma / sqrt(2 * M_PI * dataSigma2);
   for (const auto &hit : hitArray) {
      double error = model->DistanceToPattern(hit->GetPosition());
      double probInlier = probInlierCoeff * exp(-0.5 * error * error / dataSigma2);
      // if((probInlier + probOutlier)>0) sumLogLikelihood = sumLogLikelihood - log(probInlier + probOutlier);

      if (error * error < dataSigma2) {
         if ((probInlier + probOutlier) > 0)
            sumLogLikelihood = sumLogLikelihood - log(probInlier + probOutlier);
         nbInliers++;
      }
   }
   double scale = sumLogLikelihood / nbInliers;
   // std::cout <<sumLogLikelihood<< " Likelihood  " << '\n';
   if (sumLogLikelihood < 0 || std::isinf(sumLogLikelihood))
      scale = 0;

   model->SetChi2(scale);
   return nbInliers;
}

int SampleConsensus::EvaluateLmeds(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray,
                                   double distanceThreshold)
{
   std::vector<double> errorsVec;
   // Loop through point and if it is an inlier, then add the error**2 to weight
   for (const auto &hit : hitArray) {

      double error = model->DistanceToPattern(hit->GetPosition());
      error = error * error;
      if (error < (distanceThreshold * distanceThreshold))
         errorsVec.push_back(error);
   }
   model->SetChi2(ContainerManip::GetMedian(errorsVec) / errorsVec.size());
   return errorsVec.size();
}

int SampleConsensus::EvaluateWeightedRansac(AtPatterns::AtPattern *model, const std::vector<const AtHit *> &hitArray,
                                            double distanceThreshold)
{
   int nbInliers = 0;
   double totalCharge = 0;
   double weight = 0;

   for (const auto &hit : hitArray) {
      auto &pos = hit->GetPosition();
      double error = model->DistanceToPattern(pos);
      error = error * error;
      if (error < (distanceThreshold * distanceThreshold)) {
         nbInliers++;
         totalCharge += hit->GetCharge();
         weight += error * hit->GetCharge();
      }
   }
   model->SetChi2(weight / totalCharge);
   return nbInliers;
}
