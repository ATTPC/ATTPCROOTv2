#ifndef ATELOSSMODELFACTORY_H
#define ATELOSSMODELFACTORY_H

#include "AtELossModel.h"

#include <memory>
#include <tuple>
#include <vector>

class TGeoMaterial;

namespace AtTools {

/**
 * Abstract factory for creating energy loss models from ROOT geometry materials.
 *
 * Subclasses hold model-type-specific configuration (e.g. catima::Config for CATIMA)
 * and produce configured AtELossModel instances on demand for any (projectile, material)
 * combination. This allows AtSimpleSimulation to auto-create models at transport time
 * for particle species that were not explicitly registered.
 */
class AtELossModelFactory {
public:
   virtual ~AtELossModelFactory() = default;

   /**
    * Create an energy loss model for the given projectile in the given target material.
    * @param projZ       Projectile atomic number
    * @param projA       Projectile mass number
    * @param projMassAmu Projectile mass in atomic mass units
    * @param material    Target material from ROOT geometry (TGeoMaterial or TGeoMixture)
    * @return Configured model ready for use, or nullptr on failure
    */
   virtual std::shared_ptr<AtELossModel>
   CreateModel(int projZ, int projA, double projMassAmu, const TGeoMaterial *material) = 0;

   // ---- Shared utility functions for extracting material info from TGeo ----

   /**
    * Extract elemental composition from a TGeoMaterial as (A, Z, stoichiometry) tuples.
    * For TGeoMixture: weight fractions are converted to integer stoichiometry.
    * For pure TGeoMaterial: returns a single element with stoichiometry 1.
    */
   static std::vector<std::tuple<int, int, int>> ExtractComposition(const TGeoMaterial *material);

   /**
    * Convert weight fractions and atomic masses to approximate integer stoichiometry.
    * Algorithm: n_i = w_i / A_i (molar ratio), normalize by smallest, round to nearest int.
    */
   static std::vector<int>
   WeightFractionsToStoichiometry(const std::vector<double> &weights, const std::vector<double> &atomicMasses);

   /**
    * Compute effective mean ionization energy (eV) for a material using Bragg's additivity rule:
    *   ln(I_eff) = sum(f_i * Z_i/A_i * ln(I_i)) / sum(f_i * Z_i/A_i)
    * where f_i are weight fractions and I_i = 13.5*Z_i eV (Bloch approximation).
    * For a pure material, returns 13.5*Z eV directly.
    */
   static double EffectiveMeanIonization(const TGeoMaterial *material);
};

} // namespace AtTools

#endif // ATELOSSMODELFACTORY_H
