#include "AtELossManagerCATIMA.h"

#include "AtELossCATIMA.h"

#include <FairLogger.h>

#include <TGeoMaterial.h>

namespace AtTools {

AtELossManager::ModelPtr AtELossManagerCATIMA::GenerateModel(int Z, int A, double massAmu, const TGeoMaterial *material)
{
   if (material == nullptr) {
      LOG(error) << "AtELossManagerCATIMA::GenerateModel: null material";
      return nullptr;
   }

   double density = material->GetDensity(); // g/cm³
   auto composition = ExtractComposition(material);

   if (composition.empty()) {
      LOG(error) << "AtELossManagerCATIMA::GenerateModel: could not extract composition from " << material->GetName();
      return nullptr;
   }

   auto model = std::make_shared<AtELossCATIMA>(density, composition);
   model->SetProjectile(A, Z, massAmu);
   model->SetConfig(fConfig);

   LOG(info) << "AtELossManagerCATIMA: generated model for Z=" << Z << " A=" << A << " in " << material->GetName()
             << " (density=" << density << " g/cm³, " << composition.size() << " elements)";
   return model;
}

} // namespace AtTools
