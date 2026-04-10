#include "AtELossFactoryCATIMA.h"

#include "AtELossCATIMA.h"

#include <FairLogger.h>

#include <TGeoMaterial.h>

namespace AtTools {

std::shared_ptr<AtELossModel>
AtELossFactoryCATIMA::CreateModel(int projZ, int projA, double projMassAmu, const TGeoMaterial *material)
{
   if (material == nullptr) {
      LOG(error) << "AtELossFactoryCATIMA::CreateModel: null material";
      return nullptr;
   }

   double density = material->GetDensity(); // g/cm³
   auto composition = ExtractComposition(material);

   if (composition.empty()) {
      LOG(error) << "AtELossFactoryCATIMA::CreateModel: could not extract composition from " << material->GetName();
      return nullptr;
   }

   auto model = std::make_shared<AtELossCATIMA>(density, composition);
   model->SetProjectile(projA, projZ, projMassAmu);
   model->SetConfig(fConfig);

   LOG(info) << "AtELossFactoryCATIMA: created model for Z=" << projZ << " A=" << projA << " in "
             << material->GetName() << " (density=" << density << " g/cm³, " << composition.size() << " elements)";
   return model;
}

} // namespace AtTools
