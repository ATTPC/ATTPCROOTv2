#ifndef ATELOSSMANAGERBETHEBLOCH_H
#define ATELOSSMANAGERBETHEBLOCH_H

#include "AtELossManager.h"

namespace AtTools {

/**
 * Energy-loss manager that synthesizes AtELossBetheBloch models on cache-miss.
 *
 * Accepted models (via AddModel) are served first; when a particle/material pair has no
 * registration, GenerateModel() builds a Bethe-Bloch model from density, Z/A, and
 * effective mean ionization.
 *
 * Mixtures: effective Z/A via electron-density weighting, I via Bragg's additivity.
 */
class AtELossManagerBetheBloch : public AtELossManager {
protected:
   ModelPtr GenerateModel(int Z, int A, double massAmu, const TGeoMaterial *material) override;
};

} // namespace AtTools

#endif // ATELOSSMANAGERBETHEBLOCH_H
