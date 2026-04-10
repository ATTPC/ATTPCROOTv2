#ifndef ATELOSSFACTORYBETHEBLOCH_H
#define ATELOSSFACTORYBETHEBLOCH_H

#include "AtELossModelFactory.h"

namespace AtTools {

/**
 * Factory that creates AtELossBetheBloch models from ROOT geometry materials.
 *
 * For pure materials, uses Z/A/density directly.
 * For mixtures, computes effective Z/A via electron-density weighting and
 * effective mean ionization energy via Bragg's additivity rule.
 *
 * This factory is essentially stateless -- all information comes from the
 * projectile and material arguments to CreateModel().
 */
class AtELossFactoryBetheBloch : public AtELossModelFactory {
public:
   std::shared_ptr<AtELossModel>
   CreateModel(int projZ, int projA, double projMassAmu, const TGeoMaterial *material) override;
};

} // namespace AtTools

#endif // ATELOSSFACTORYBETHEBLOCH_H
