#include "AtELossManagerBetheBloch.h"

#include "AtELossBetheBloch.h"

#include <FairLogger.h>

#include <TGeoMaterial.h>

#include <algorithm>
#include <cmath>

namespace AtTools {

namespace {
constexpr double kAmuToMeV = 931.494; // MeV/c² per amu
} // namespace

AtELossManager::ModelPtr
AtELossManagerBetheBloch::GenerateModel(int Z, int A, double massAmu, const TGeoMaterial *material)
{
   if (material == nullptr) {
      LOG(error) << "AtELossManagerBetheBloch::GenerateModel: null material";
      return nullptr;
   }

   double density = material->GetDensity(); // g/cm³
   double projMass = massAmu * kAmuToMeV;

   const auto *mixture = dynamic_cast<const TGeoMixture *>(material);
   if (mixture == nullptr) {
      int matZ = static_cast<int>(std::round(material->GetZ()));
      int matA = static_cast<int>(std::round(material->GetA()));
      double I_eV = 13.5 * matZ;

      auto model = std::make_shared<AtELossBetheBloch>(Z, projMass, matZ, matA, density, I_eV);
      LOG(info) << "AtELossManagerBetheBloch: generated model for Z=" << Z << " A=" << A << " in "
                << material->GetName() << " (pure Z=" << matZ << ", density=" << density << " g/cm³)";
      return model;
   }

   int nElem = mixture->GetNelements();
   double sumWZoverA = 0;
   double sumWoverA = 0;

   for (int i = 0; i < nElem; ++i) {
      double w = mixture->GetWmixt()[i];
      double z = mixture->GetZmixt()[i];
      double a = mixture->GetAmixt()[i];
      if (a <= 0)
         continue;
      sumWZoverA += w * z / a;
      sumWoverA += w / a;
   }

   if (sumWoverA <= 0) {
      LOG(error) << "AtELossManagerBetheBloch::GenerateModel: invalid mixture composition";
      return nullptr;
   }

   int effZ = std::max(1, static_cast<int>(std::round(sumWZoverA / sumWoverA)));
   int effA = std::max(1, static_cast<int>(std::round(1.0 / sumWoverA)));
   double I_eV = EffectiveMeanIonization(material);

   auto model = std::make_shared<AtELossBetheBloch>(Z, projMass, effZ, effA, density, I_eV);
   LOG(info) << "AtELossManagerBetheBloch: generated model for Z=" << Z << " A=" << A << " in " << material->GetName()
             << " (mixture, eff Z=" << effZ << " A=" << effA << " I=" << I_eV << " eV, density=" << density
             << " g/cm³)";
   return model;
}

} // namespace AtTools
