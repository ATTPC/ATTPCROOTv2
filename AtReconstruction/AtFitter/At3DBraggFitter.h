#ifndef AT3DBRAGGFITTER_H
#define AT3DBRAGGFITTER_H

#include "At3DBraggFitResult.h"
#include "AtFittedTrack.h"
#include "AtFitter.h"
#include "AtPatternLine.h"
#include "AtTrack.h"

#include <TH1F.h>

#include <catima/catima.h>

namespace AtFITTER {

static TH1F *gHistExpBraggCurve{nullptr};
static catima::Material *gMaterial{nullptr};
static catima::Projectile *gProjectile{nullptr};
static double gMassUma{1};
static double gDS{1};

class At3DBraggFitter : public AtFitter {
protected:
   int fParticleIdx{0};
   std::vector<std::pair<Int_t, Int_t>> fAZPairs{};
   std::vector<double> fParticleMassesUma{};

   Double_t fMaterialDensity{0};
   std::vector<std::tuple<Int_t, Int_t, Int_t>> fMaterialComponents{};

   double fDZ{1};
   double fMinKE{0.5};
   double fMaxKE{15};
   double fKEGuess{2};

public:
   At3DBraggFitter() = default;
   ~At3DBraggFitter() = default;

   void Init() override;

   std::vector<std::unique_ptr<AtFittedTrack>> ProcessTracks(std::vector<AtTrack> &tracks) override;

   std::unique_ptr<At3DBraggFitResult> ProcessTrack(AtTrack &track);

   void AddAZPair(std::pair<Int_t, Int_t> pair) { fAZPairs.push_back(pair); }
   void AddMassUma(double value) { fParticleMassesUma.push_back(value); }

   void SetMaterialDensity(Double_t value) { fMaterialDensity = value; }
   void AddMaterialComponent(std::tuple<Int_t, Int_t, Int_t> component) { fMaterialComponents.push_back(component); }
   void SetProjectileIndex(Int_t i);

   void SetDZ(double value) { fDZ = value; }
   void SetMinKE(double value) { fMinKE = value; }
   void SetMaxKE(double value) { fMaxKE = value; }
   void SetKEGuess(double value) { fKEGuess = value; }

   static std::vector<std::pair<double, double>> GetDeDx(Double_t kineticEnergy);
   static std::vector<std::pair<double, double>> GetELossModel(Double_t kineticEnergy);

protected:
   static void BraggFCN(int &npar, double *gin, double &fval, double *par, int iflag);

   Double_t EstimateEnergyFromRange(Double_t range);
   Double_t EstimateAmplitudeFactor(Double_t kineticEnergy);

   void SetProjectile(catima::Projectile *projectile) { AtFITTER::gProjectile = projectile; }

   ClassDefOverride(At3DBraggFitter, 1);
};

} // namespace AtFITTER

#endif
