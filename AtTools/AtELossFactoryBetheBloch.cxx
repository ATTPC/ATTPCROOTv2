#include "AtELossFactoryBetheBloch.h"

#include "AtELossBetheBloch.h"

#include <FairLogger.h>

#include <TGeoMaterial.h>

#include <cmath>

namespace AtTools {

namespace {
constexpr double kAmuToMeV = 931.494; // MeV/c² per amu
} // namespace

std::shared_ptr<AtELossModel>
AtELossFactoryBetheBloch::CreateModel(int projZ, int projA, double projMassAmu, const TGeoMaterial *material)
{
   if (material == nullptr) {
      LOG(error) << "AtELossFactoryBetheBloch::CreateModel: null material";
      return nullptr;
   }

   double density = material->GetDensity(); // g/cm³
   double projMass = projMassAmu * kAmuToMeV;

   const auto *mixture = dynamic_cast<const TGeoMixture *>(material);
   if (mixture == nullptr) {
      // Pure material
      int matZ = static_cast<int>(std::round(material->GetZ()));
      int matA = static_cast<int>(std::round(material->GetA()));
      double I_eV = 13.5 * matZ; // Bloch approximation

      auto model = std::make_shared<AtELossBetheBloch>(projZ, projMass, matZ, matA, density, I_eV);
      LOG(info) << "AtELossFactoryBetheBloch: created model for Z=" << projZ << " A=" << projA << " in "
                << material->GetName() << " (pure Z=" << matZ << ", density=" << density << " g/cm³)";
      return model;
   }

   // Mixture: compute effective Z and A via electron-density weighting
   //   <Z/A> = sum(w_i * Z_i / A_i)
   //   <Z> = sum(w_i * Z_i / A_i) / sum(w_i / A_i)
   //   <A> = sum(w_i) / sum(w_i / A_i) = 1 / sum(w_i / A_i)  [since sum(w_i)=1]
   int nElem = mixture->GetNelements();
   double sumWZoverA = 0;
   double sumWoverA = 0;

   for (int i = 0; i < nElem; ++i) {
      double w = mixture->GetWmixt()[i];
      double Z = mixture->GetZmixt()[i];
      double A = mixture->GetAmixt()[i];
      if (A <= 0)
         continue;
      sumWZoverA += w * Z / A;
      sumWoverA += w / A;
   }

   if (sumWoverA <= 0) {
      LOG(error) << "AtELossFactoryBetheBloch::CreateModel: invalid mixture composition";
      return nullptr;
   }

   int effZ = static_cast<int>(std::round(sumWZoverA / sumWoverA));
   int effA = static_cast<int>(std::round(1.0 / sumWoverA));
   double I_eV = EffectiveMeanIonization(material);

   // Ensure effective values are at least 1
   effZ = std::max(effZ, 1);
   effA = std::max(effA, 1);

   auto model = std::make_shared<AtELossBetheBloch>(projZ, projMass, effZ, effA, density, I_eV);
   LOG(info) << "AtELossFactoryBetheBloch: created model for Z=" << projZ << " A=" << projA << " in "
             << material->GetName() << " (mixture, eff Z=" << effZ << " A=" << effA << " I=" << I_eV
             << " eV, density=" << density << " g/cm³)";
   return model;
}

} // namespace AtTools
