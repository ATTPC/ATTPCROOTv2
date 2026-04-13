#ifndef ATELOSSMANAGERCATIMA_H
#define ATELOSSMANAGERCATIMA_H

#include "AtELossManager.h"

#include <catima/config.h>

namespace AtTools {

/**
 * Energy-loss manager that synthesizes AtELossCATIMA models on cache-miss.
 *
 * Applies a user-configured catima::Config to every generated model so options like
 * z_effective choice or calculation method are consistent across the simulation.
 */
class AtELossManagerCATIMA : public AtELossManager {
public:
   void SetConfig(catima::Config cfg) { fConfig = cfg; }
   catima::Config GetConfig() const { return fConfig; }

protected:
   ModelPtr GenerateModel(int Z, int A, double massAmu, const TGeoMaterial *material) override;

private:
   catima::Config fConfig{catima::default_config};
};

} // namespace AtTools

#endif // ATELOSSMANAGERCATIMA_H
