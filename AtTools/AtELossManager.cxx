#include "AtELossManager.h"

#include <FairLogger.h>

#include <TGeoMaterial.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace AtTools {

void AtELossManager::AddModel(int Z, int A, ModelPtr model)
{
   fRegistered[{Z, A, std::string{}}] = std::move(model);
}

void AtELossManager::AddModel(int Z, int A, const std::string &materialName, ModelPtr model)
{
   fRegistered[{Z, A, materialName}] = std::move(model);
}

AtELossManager::ModelPtr AtELossManager::GetModel(int Z, int A, double massAmu, const TGeoMaterial *material)
{
   const std::string materialName = material != nullptr ? material->GetName() : std::string{};

   // 1. Material-specific registration.
   if (!materialName.empty()) {
      auto it = fRegistered.find({Z, A, materialName});
      if (it != fRegistered.end())
         return it->second;
   }

   // 2. Material-agnostic registration.
   auto agnostic = fRegistered.find({Z, A, std::string{}});
   if (agnostic != fRegistered.end())
      return agnostic->second;

   // 3. Auto-generated cache (keyed by material).
   if (!materialName.empty()) {
      auto cached = fCache.find({Z, A, materialName});
      if (cached != fCache.end())
         return cached->second;

      // 4. Attempt generation and cache the result.
      auto fresh = GenerateModel(Z, A, massAmu, material);
      if (fresh) {
         fCache[{Z, A, materialName}] = fresh;
         return fresh;
      }
   }

   return nullptr;
}

void AtELossManager::ClearCache()
{
   fCache.clear();
}

std::vector<std::tuple<int, int, int>> AtELossManager::ExtractComposition(const TGeoMaterial *material)
{
   if (material == nullptr) {
      LOG(error) << "AtELossManager::ExtractComposition: null material";
      return {};
   }

   const auto *mixture = dynamic_cast<const TGeoMixture *>(material);
   if (mixture == nullptr) {
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

std::vector<int> AtELossManager::WeightFractionsToStoichiometry(const std::vector<double> &weights,
                                                                const std::vector<double> &atomicMasses)
{
   if (weights.size() != atomicMasses.size() || weights.empty())
      return {};

   std::vector<double> molar(weights.size());
   for (size_t i = 0; i < weights.size(); ++i) {
      if (atomicMasses[i] <= 0) {
         LOG(error) << "AtELossManager::WeightFractionsToStoichiometry: non-positive atomic mass";
         return {};
      }
      molar[i] = weights[i] / atomicMasses[i];
   }

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

double AtELossManager::EffectiveMeanIonization(const TGeoMaterial *material)
{
   if (material == nullptr)
      return 0;

   const auto *mixture = dynamic_cast<const TGeoMixture *>(material);
   if (mixture == nullptr) {
      int Z = static_cast<int>(std::round(material->GetZ()));
      return 13.5 * Z; // eV, Bloch approximation
   }

   int nElem = mixture->GetNelements();
   double numerator = 0;
   double denominator = 0;

   for (int i = 0; i < nElem; ++i) {
      double w = mixture->GetWmixt()[i];
      double Z = mixture->GetZmixt()[i];
      double A = mixture->GetAmixt()[i];
      if (A <= 0 || Z <= 0)
         continue;

      double I_i = 13.5 * Z; // eV, Bloch approximation per element
      double frac = w * Z / A;
      numerator += frac * std::log(I_i);
      denominator += frac;
   }

   if (denominator <= 0)
      return 13.5;

   return std::exp(numerator / denominator);
}

} // namespace AtTools
