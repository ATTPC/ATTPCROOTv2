#include "AtRandomSample.h"

#include "AtHit.h"

#include <FairLogger.h>

#include <Math/Vector3D.h> // for DisplacementVector3D
#include <TRandom.h>       // for TRandom
#include <TRandom3.h>

#include <algorithm> // for max, find
#include <cmath>     // for exp, pow, sqrt
#include <memory>    // for allocator_traits<>::value_type
#include <numeric>
#include <ostream> // for operator<<

/**
 * Implementation class for random sampling
 */
class AtRandomSample::AtRandomSample_impl {
public:
   static std::vector<XYZPoint> positionFromIndices(const std::vector<int> inds, const std::vector<AtHit> &hits)
   {
      std::vector<XYZPoint> ret;
      for (auto ind : inds)
         ret.push_back(hits[ind].GetPosition());

      return ret;
   }
   static bool isInVector(const std::vector<int> inds, int val)
   {
      return std::find(inds.begin(), inds.end(), val) != inds.end();
   }

   static std::vector<XYZPoint> sampleUniform(int N, const std::vector<AtHit> &hits)
   {
      std::vector<int> sampled;
      sampled.push_back(gRandom->Uniform(0, hits.size()));
      while (sampled.size() < N) {
         int candIndex = 0;

         do {
            candIndex = gRandom->Uniform(0, hits.size());
         } while (isInVector(sampled, candIndex));
         sampled.push_back(candIndex);
      }

      return positionFromIndices(sampled, hits);
   }

   static bool checkGauss(XYZPoint pos1, XYZPoint pos2, double sigma)
   {
      auto dist = std::sqrt((pos1 - pos2).Mag2());
      auto gauss = 1.0 * exp(-1.0 * pow(dist / sigma, 2.0));
      auto y = gRandom->Uniform(0, 1);
      return y <= gauss;
   }
   static std::vector<XYZPoint> sampleGaussian(int N, const std::vector<AtHit> &hits)
   {
      double sigma = 30.0;

      std::vector<int> sampled;
      sampled.push_back(gRandom->Uniform(0, hits.size()));
      auto iniPos = hits.at(sampled.back()).GetPosition();

      for (int i = 1; i < N; ++i) {
         int candIndex = 0;
         int count = 0;
         bool validPoint = false;

         do {
            count++;
            candIndex = gRandom->Uniform(0, hits.size());
            auto candPos = hits.at(candIndex).GetPosition();

            if (count > 20 && !isInVector(sampled, candIndex))
               break;
            validPoint = checkGauss(iniPos, candPos, sigma);

         } while (isInVector(sampled, candIndex) || !validPoint);

         sampled.push_back(candIndex);
      }
      return positionFromIndices(sampled, hits);
   }

   static bool checkCharge(int ind, const std::vector<double> &pdf, double avgCharge)
   {
      return (pdf[ind] >= gRandom->Uniform(0, 2 * avgCharge));
   }

   static std::vector<XYZPoint> sampleWeighted(int N, const std::vector<AtHit> &hits)
   {
      double avgCharge = 0;
      auto Proba = getPDF(avgCharge, hits);

      std::vector<int> sampled;
      sampled.push_back(gRandom->Uniform(0, hits.size()));
      auto iniPos = hits.at(sampled.back()).GetPosition();

      while (sampled.size() < N) {
         bool validPoint = false;
         int counter = 0;
         int candInd;

         do {
            counter++;
            candInd = gRandom->Uniform(0, hits.size());

            if (counter > 30 && !isInVector(sampled, candInd))
               break;

            validPoint = checkCharge(candInd, Proba, avgCharge);

         } while (isInVector(sampled, candInd) || !validPoint);
         sampled.push_back(candInd);
      }
      return positionFromIndices(sampled, hits);
   }

   static std::vector<XYZPoint> sampleWeightedGaussian(int N, const std::vector<AtHit> &hits)
   {
      //-------Weighted sampling + Gauss dist.
      double avgCharge = 0;
      auto Proba = getPDF(avgCharge, hits);
      double sigma = 30.0;

      std::vector<int> sampled;
      sampled.push_back(gRandom->Uniform(0, hits.size()));
      auto iniPos = hits.at(sampled.back()).GetPosition();

      while (sampled.size() < N) {
         bool validPoint = false;
         int counter = 0;
         int candInd;

         do {
            counter++;
            candInd = gRandom->Uniform(0, hits.size());
            auto candPos = hits.at(candInd).GetPosition();

            if (counter > 30 && !isInVector(sampled, candInd))
               break;
            validPoint = checkGauss(iniPos, candPos, sigma);
            validPoint &= checkCharge(candInd, Proba, avgCharge);

         } while (isInVector(sampled, candInd) || !validPoint);
         sampled.push_back(candInd);
      }
      return positionFromIndices(sampled, hits);
   }

   static std::vector<double> getPDF(double &avgCharge, const std::vector<AtHit> &hits)
   {
      double Tcharge =
         std::accumulate(hits.begin(), hits.end(), 0, [](double sum, auto &a) { return sum + a.GetCharge(); });

      avgCharge = Tcharge / hits.size();
      std::vector<double> w;
      if (Tcharge > 0)
         for (const auto &hit : hits)
            w.push_back(hit.GetCharge() / Tcharge);

      return w;
   }
};

std::ostream &operator<<(std::ostream &os, const AtRandomSample::SampleMethod &t)
{
   using method = AtRandomSample::SampleMethod;
   switch (t) {
   case (method::kUniform): os << "SampleMethod::kUniform"; break;
   case (method::kGaussian): os << "SampleMethod::kGaussian"; break;
   case (method::kWeighted): os << "SampleMethod::kWeighted"; break;
   case (method::kWeightedGaussian): os << "SampleMethod::kWeightedGaussian"; break;
   default: os << "SampleMethod::Other";
   }
   return os;
}

/**
 * @brief Sample points from hits
 *
 * Sample N unique points from hits using the method mode
 *
 * @todo Generalize the functions being called so they return and arbitrary number
 * of points based on the requirements of the model
 *
 * @param[in] mode flag instructing what sampling algorithm to use
 * @return vector of sampled points
 */
std::vector<AtRandomSample::XYZPoint>
AtRandomSample::SamplePoints(int N, const std::vector<AtHit> &hits, SampleMethod mode = SampleMethod::kUniform)
{
   switch (mode) {
   case (SampleMethod::kUniform): return AtRandomSample_impl::sampleUniform(N, hits);
   case (SampleMethod::kGaussian): return AtRandomSample_impl::sampleGaussian(N, hits);
   case (SampleMethod::kWeighted): return AtRandomSample_impl::sampleWeighted(N, hits);
   case (SampleMethod::kWeightedGaussian): return AtRandomSample_impl::sampleWeightedGaussian(N, hits);
   default: LOG(error) << "Invalid sample method passed " << mode; return {};
   }
}
