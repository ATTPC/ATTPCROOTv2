#include "AtSample.h"

#include "AtContainerManip.h"
#include "AtHit.h"

#include <Math/Point3D.h> // for PositionVector3D
#include <TRandom.h>      // for TRandom
#include <TRandom3.h>

#include <functional> // for multiplies
#include <numeric>

using namespace RandomSample;

/**
 * @brief Sample hits (AtHit) from fHits.
 *
 * Assumes fCDF is already setup and we are just using it.
 */
std::vector<AtHit> AtSample::SampleHits(int N)
{
   // Using the sampled indices, return a vector of positions
   std::vector<AtHit> ret;
   for (auto ind : sampleIndicesFromCDF(N))
      ret.push_back(*fHits->at(ind));
   return ret;
}

/**
 * @brief Sample spacial locations (XYZPoints) from fHits.
 *
 * Calls SampleHits(int N).
 */
std::vector<ROOT::Math::XYZPoint> AtSample::SamplePoints(int N)
{
   std::vector<ROOT::Math::XYZPoint> ret;
   for (const auto &hit : SampleHits(N))
      ret.push_back(hit.GetPosition());
   return ret;
}

/**
 * @brief Get the index i where CDF[i] >= r and CDF[i-1] < r.
 *
 * Loop through fCDF, skipping any index in vetoed, for the index that corresponds to r
 * So need to track what indices are not allowed (vetoed) and the amount of the CDF that has been removed (i.e.
 * sum of PDFs for all removed indices). (rmCDF) The new CDF is then oldCDF/(1-rmCDF) We then need to loop through
 * CDF and look for index where CDF[i] >= r

 * @param[in] r Random number between [0,1) to compare to fCDF
 * @param[in] vetoed Indices to ignore while searching fCDF
 *
 */
int AtSample::getIndexFromCDF(double r, double rmCFD, std::vector<int> vetoed)
{
   double probRemoved = 0; // Probability removed up to this point in the CFD
   for (int i = 0; i < fCDF.size(); ++i) {

      if (isInVector(i, vetoed)) {
         probRemoved += getPDFfromCDF(i);
         continue;
      }

      if ((fCDF[i] - probRemoved) / (1.0 - rmCFD) >= r)
         return i;
   }
   return fCDF.size() - 1;
}

/**
 * Get a list of indices aampled according to the constructed fCDF (assumes FillCDF() has been called already)
 *
 * @param[in] N number of pcoints to sample. If sampling without replacement this should be small compared to the
 * total number of hits in the hit array.
 * @param[in] vetoed Indices to not sample (even if sampling with replacement).
 */
std::vector<int> AtSample::sampleIndicesFromCDF(int N, std::vector<int> vetoed)
{
   // Get the total probablility from the CDF that accounts for the vetoed pads
   auto probSum = [this](double accum, int ind) { return accum + getPDFfromCDF(ind); };
   double rmProb = std::accumulate(vetoed.begin(), vetoed.end(), 0.0, probSum);

   std::vector<int> sampledInd;
   while (sampledInd.size() < N) {
      auto r = gRandom->Uniform();

      // Get the index i where CDF[i] >= r and CDF[i-1] < r
      int hitInd = getIndexFromCDF(r, rmProb, vetoed);

      if (!fWithReplacement) {
         rmProb += getPDFfromCDF(hitInd);
         vetoed.push_back(hitInd);
      }

      // Save the index if it has not already been sampled
      if (fWithReplacement || !isInVector(hitInd, sampledInd)) {
         sampledInd.push_back(hitInd);
      }
   }

   return sampledInd;
}

double AtSample::getPDFfromCDF(int index)
{
   return index == 0 ? fCDF[0] : fCDF[index] - fCDF[index - 1];
}
/**
 * Fill the cumulitive distribution function to sample using the marginal PDFs returned by the
 * function PDF(const AtHit &hit) from every entry in the vector fHits.
 *
 * Each marginal PDF is normalized to construct the final CDF. Assumes the marginal PDFs are independent
 * (i.e. the joint PDF is just the product of all marginal PDFs)
 */
void AtSample::FillCDF()
{
   std::vector<double> normalization;
   fCDF.clear();
   for (const auto &hit : *fHits) {

      // Get the unnormalized marginal and joint PDFs
      auto pdfMarginal = PDF(*hit);
      auto pdfJoint = std::accumulate(pdfMarginal.begin(), pdfMarginal.end(), 1.0,
                                      std::multiplies<>()); // Has to be 1.0 not 1 or return type is deduced as int

      // If this is the first hit, setup the normalization vector
      if (normalization.size() == 0)
         normalization.assign(pdfMarginal.size(), 0);

      for (int i = 0; i < pdfMarginal.size(); ++i)
         normalization[i] += pdfMarginal[i];

      if (fCDF.size() == 0)
         fCDF.push_back(pdfJoint);
      else
         fCDF.push_back(pdfJoint + fCDF.back());
   }

   // Get the total normalization from the marginal PDFs, and normalize the CDF
   auto norm = std::accumulate(normalization.begin(), normalization.end(), 1.0, std::multiplies<>());
   for (auto &elem : fCDF) {
      elem /= norm;
   }
}

void AtSample::SetHitsToSample(const std::vector<HitPtr> &hits)
{
   SetHitsToSample(ContainerManip::GetConstPointerVector(hits));
}

void AtSample::SetHitsToSample(const std::vector<AtHit> &hits)
{
   // Cast away const, for container manip but it is restored when SetHitsToSample is called
   SetHitsToSample(ContainerManip::GetConstPointerVector(hits));
}
