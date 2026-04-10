#include "AtELossModelFactory.h"

#include <FairLogger.h>

#include <TGeoMaterial.h>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace AtTools {

std::vector<std::tuple<int, int, int>> AtELossModelFactory::ExtractComposition(const TGeoMaterial *material)
{
   if (material == nullptr) {
      LOG(error) << "AtELossModelFactory::ExtractComposition: null material";
      return {};
   }

   const auto *mixture = dynamic_cast<const TGeoMixture *>(material);
   if (mixture == nullptr) {
      // Pure material: single element
      int Z = static_cast<int>(std::round(material->GetZ()));
      int A = static_cast<int>(std::round(material->GetA()));
      return {{A, Z, 1}};
   }

   int nElem = mixture->GetNelements();
   if (nElem <= 0)
      return {};

   std::vector<double> weights(nElem);
   std::vector<double> masses(nElem);
   for (int i = 0; i < nElem; ++i) {
      weights[i] = mixture->GetWmixt()[i];
      masses[i] = mixture->GetAmixt()[i];
   }

   auto stoich = WeightFractionsToStoichiometry(weights, masses);

   std::vector<std::tuple<int, int, int>> result;
   result.reserve(nElem);
   for (int i = 0; i < nElem; ++i) {
      int A = static_cast<int>(std::round(mixture->GetAmixt()[i]));
      int Z = static_cast<int>(std::round(mixture->GetZmixt()[i]));
      result.emplace_back(A, Z, stoich[i]);
   }
   return result;
}

std::vector<int>
AtELossModelFactory::WeightFractionsToStoichiometry(const std::vector<double> &weights,
                                                     const std::vector<double> &atomicMasses)
{
   if (weights.size() != atomicMasses.size() || weights.empty())
      return {};

   // Compute molar ratios: n_i = w_i / A_i
   std::vector<double> molar(weights.size());
   for (size_t i = 0; i < weights.size(); ++i) {
      if (atomicMasses[i] <= 0) {
         LOG(error) << "AtELossModelFactory::WeightFractionsToStoichiometry: non-positive atomic mass";
         return {};
      }
      molar[i] = weights[i] / atomicMasses[i];
   }

   // Normalize by smallest non-zero molar ratio
   double minMolar = *std::min_element(molar.begin(), molar.end(), [](double a, double b) {
      if (a <= 0)
         return false;
      if (b <= 0)
         return true;
      return a < b;
   });

   if (minMolar <= 0)
      return std::vector<int>(weights.size(), 1);

   std::vector<int> stoich(weights.size());
   for (size_t i = 0; i < molar.size(); ++i) {
      stoich[i] = std::max(1, static_cast<int>(std::round(molar[i] / minMolar)));
   }
   return stoich;
}

double AtELossModelFactory::EffectiveMeanIonization(const TGeoMaterial *material)
{
   if (material == nullptr)
      return 0;

   const auto *mixture = dynamic_cast<const TGeoMixture *>(material);
   if (mixture == nullptr) {
      // Pure material: Bloch approximation
      int Z = static_cast<int>(std::round(material->GetZ()));
      return 13.5 * Z; // eV
   }

   // Bragg additivity: ln(I_eff) = sum(f_i * Z_i/A_i * ln(I_i)) / sum(f_i * Z_i/A_i)
   int nElem = mixture->GetNelements();
   double numerator = 0;
   double denominator = 0;

   for (int i = 0; i < nElem; ++i) {
      double w = mixture->GetWmixt()[i];
      double Z = mixture->GetZmixt()[i];
      double A = mixture->GetAmixt()[i];
      if (A <= 0 || Z <= 0)
         continue;

      double I_i = 13.5 * Z; // Bloch approximation per element, eV
      double frac = w * Z / A;
      numerator += frac * std::log(I_i);
      denominator += frac;
   }

   if (denominator <= 0)
      return 13.5; // fallback

   return std::exp(numerator / denominator);
}

} // namespace AtTools
