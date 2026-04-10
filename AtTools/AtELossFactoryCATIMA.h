#ifndef ATELOSSFACTORYCATIMA_H
#define ATELOSSFACTORYCATIMA_H

#include "AtELossModelFactory.h"

#include <catima/config.h>

namespace AtTools {

/**
 * Factory that creates AtELossCATIMA models from ROOT geometry materials.
 *
 * Stores a catima::Config that is applied to every model it creates. This allows
 * the user to configure CATIMA options (z_effective model, calculation method, etc.)
 * once, and have them consistently applied to all auto-created models.
 */
class AtELossFactoryCATIMA : public AtELossModelFactory {
   catima::Config fConfig{catima::default_config};

public:
   void SetConfig(catima::Config cfg) { fConfig = cfg; }
   catima::Config GetConfig() const { return fConfig; }

   std::shared_ptr<AtELossModel>
   CreateModel(int projZ, int projA, double projMassAmu, const TGeoMaterial *material) override;
};

} // namespace AtTools

#endif // ATELOSSFACTORYCATIMA_H
