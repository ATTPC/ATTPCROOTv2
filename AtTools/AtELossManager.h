#ifndef ATELOSSMANAGER_H
#define ATELOSSMANAGER_H

#include "AtELossModel.h"

#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

class TGeoMaterial;

namespace AtTools {

/**
 * Energy-loss model manager: accepts pre-built models and serves them to transport code.
 *
 * The base class is "accept-only": users register models with AddModel(); GetModel() serves
 * the registered ones and returns nullptr otherwise. Subclasses override GenerateModel() to
 * synthesize a model on cache-miss (see AtELossManagerBetheBloch, AtELossManagerCATIMA).
 *
 * Models can be registered material-agnostically (AddModel(Z, A, model)) or material-
 * specifically (AddModel(Z, A, materialName, model)). Lookup prefers material-specific
 * over material-agnostic, so MCFission / AtMCFitter callers that pass a single (Z, A)
 * table without material context continue to work, while validation macros that want
 * different models per material can be precise.
 *
 * Static helpers (ExtractComposition, WeightFractionsToStoichiometry,
 * EffectiveMeanIonization) translate TGeoMaterial/TGeoMixture into the forms needed
 * by concrete generators and are used by the BetheBloch and CATIMA subclasses.
 */
class AtELossManager {
public:
   using ModelPtr = std::shared_ptr<AtELossModel>;

   virtual ~AtELossManager() = default;

   /// Register a pre-built model for (Z, A), served regardless of material.
   /// Use this for SRIM/LISE tables or legacy single-model workflows (MCFission, AtMCFitter).
   void AddModel(int Z, int A, ModelPtr model);

   /// Register a pre-built model for (Z, A) in a specific material.
   /// Takes priority over material-agnostic registrations and over GenerateModel().
   void AddModel(int Z, int A, const std::string &materialName, ModelPtr model);

   /// Look up a model for (Z, A) in the given material.
   /// Order: material-specific registration → material-agnostic registration →
   /// auto-generated cache → GenerateModel() (cached on first call) → nullptr.
   /// @param material may be nullptr only if a material-agnostic registration exists.
   ModelPtr GetModel(int Z, int A, double massAmu, const TGeoMaterial *material);

   /// Drop models produced by GenerateModel(); manually registered models stay.
   void ClearCache();

   // ---- Shared utility functions for extracting material info from TGeo ----

   /// Extract elemental composition as (A, Z, stoichiometry) tuples.
   static std::vector<std::tuple<int, int, int>> ExtractComposition(const TGeoMaterial *material);

   /// Convert weight fractions and atomic masses to approximate integer stoichiometry.
   static std::vector<int>
   WeightFractionsToStoichiometry(const std::vector<double> &weights, const std::vector<double> &atomicMasses);

   /// Compute effective mean ionization energy (eV) using Bragg's additivity rule.
   static double EffectiveMeanIonization(const TGeoMaterial *material);

protected:
   /// Override to synthesize a model from physics parameters. Default returns nullptr,
   /// i.e. the base class is accept-only. Subclasses (BetheBloch, CATIMA) override.
   virtual ModelPtr GenerateModel(int /*Z*/, int /*A*/, double /*massAmu*/, const TGeoMaterial * /*material*/)
   {
      return nullptr;
   }

private:
   // (Z, A, materialName) — materialName == "" for material-agnostic entries.
   using Key = std::tuple<int, int, std::string>;

   std::map<Key, ModelPtr> fRegistered; ///< From AddModel(...)
   std::map<Key, ModelPtr> fCache;      ///< From GenerateModel(...)
};

} // namespace AtTools

#endif // ATELOSSMANAGER_H
