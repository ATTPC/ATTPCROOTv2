#include "AtkNN.h"

#include "AtHit.h"
namespace AtTools {
namespace DataCleaning {

HitCloud AtkNN::CleanData(const HitCloud &hits)
{
   HitCloud ret;
   for (const auto &hit : hits) {
      if (kNN(hits, *hit))
         ret.push_back(std::make_unique<AtHit>(*hit));
   }
   return ret;
}

/**
 * @brief kNN algorithm to clean data.
 *
 * Returns true of k-nearest neighbors to hitRef are within a threshold distance.
 */
bool AtkNN::kNN(const std::vector<std::unique_ptr<AtHit>> &hits, AtHit &hitRef)
{
   int k = fkNN;
   std::vector<Double_t> distances;
   distances.reserve(hits.size());

   std::for_each(hits.begin(), hits.end(), [&distances, &hitRef](const std::unique_ptr<AtHit> &hit) {
      auto dist = (hitRef.GetPosition() - hit->GetPosition()).Mag2();
      if (dist > 0.01) // Ignore self
         distances.push_back(std::sqrt(dist));
   });

   std::sort(distances.begin(), distances.end(), [](Double_t a, Double_t b) { return a < b; });

   /*
  for (auto i = 0; i < distances.size(); ++i)
     std::cout << distances.at(i) << " ";
  std::cout << "\n";
  */

   Double_t mean = 0.0;
   Double_t stdDev = 0.0;

   if (k > hits.size())
      k = hits.size();

   // Compute mean distance of kNN
   for (auto i = 0; i < k; ++i)
      mean += distances.at(i);

   mean /= k;

   // Compute threshold
   Double_t T = mean;
   /*
   std::cout << "For hit at " << hitRef.GetPosition() << " T: " << T << " fkNNDist: " << fkNNDist << " k: " << k
             << "\n";
             */

   return T < fkNNDist;
}

} // namespace DataCleaning
} // namespace AtTools