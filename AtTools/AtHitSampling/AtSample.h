#ifndef ATHITSAMPLER_H
#define ATHITSAMPLER_H

#include <Math/Point3Dfwd.h> // for XYZPoint

#include <algorithm>
#include <memory>
#include <vector>

class AtHit;

/**
 * @brief Classes for sampling AtHits.
 *
 * Group of classes for randomly sampling AtHits according to some probability density function (PDF).
 *
 * Provides subclassed interfaces for the case where the hits sampled are independent (\ref AtIndependentSample) and
 * when the the PDF depends on some reference hit (\ref AtSampleFromReference).
 *
 * To add an additional sampling method, at minimum it must inherit \ref AtSample. It should also be added to the
 * SampleMethod enum, and the static factory method AtSample::CreateSampler.
 *
 * @defgroup AtHitSampling Random Sampling
 */
namespace RandomSample {
enum class SampleMethod;

/**
 * @brief Interface for randomly sampling AtHits.
 *
 * Samples according to the cumulitive distribution function fCDF.
 *
 * @ingroup AtHitSampling
 */
class AtSample {
protected:
   using HitPtr = std::unique_ptr<AtHit>;
   const std::vector<const AtHit *> *fHits; //< Hits to sample from
   std::vector<double> fCDF;                //< Cummulative distribution function for hits
   bool fWithReplacement{false};            //< If we should sample with replacement

public:
   virtual ~AtSample() = default;

   virtual std::vector<AtHit> SampleHits(int N);
   std::vector<ROOT::Math::XYZPoint> SamplePoints(int N);

   virtual void SetHitsToSample(const std::vector<const AtHit *> &hits) = 0;
   [[deprecated]] void SetHitsToSample(const std::vector<HitPtr> &hits);
   [[deprecated]] void SetHitsToSample(const std::vector<AtHit> &hits);

   void SetSampleWithReplacement(bool val) { fWithReplacement = val; }

protected:
   /**
    * Computes the unnormalized marginal PDFs at the hit.
    *
    * For example, a charge-weighted and spacial gaussian would looke like
    * `return {charge, gaussian_pdf(distance, sigma)};`
    *
    * @return vector where each element is a different maginal pdf
    */
   virtual std::vector<double> PDF(const AtHit &hit) = 0;
   void FillCDF();

   std::vector<int> sampleIndicesFromCDF(int N, std::vector<int> vetoed = {});
   int getIndexFromCDF(double r, double rmCFD, std::vector<int> vetoed);

   double getPDFfromCDF(int index);

   template <typename T>
   static inline bool isInVector(T val, std::vector<T> vec)
   {
      if (vec.size() == 0)
         return false;
      return std::find(vec.begin(), vec.end(), val) != vec.end();
   }
};
} // namespace RandomSample

#endif //#ifndef ATHITSAMPLER_H
