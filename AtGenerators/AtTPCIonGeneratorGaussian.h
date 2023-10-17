#ifndef AtTPCIONGENERAtORGAUSSIAN_H
#define AtTPCIONGENERAtORGAUSSIAN_H

#include "AtTPCIonGenerator.h"

#include <Rtypes.h> // for Double_t, Int_t, THashConsistencyHolder

class TBuffer;
class TClass;
class TMemberInspector;

class AtTPCIonGeneratorGaussian : public AtTPCIonGenerator {
protected:
   Double_t fTheta{0}; //< Maximum angle [rad]
   Double_t fX{0};     //< X coordinate at the window
   Double_t fY{0};     //< Y coordinate at the window

   virtual void SetVertexCoordinates() override;

public:
   AtTPCIonGeneratorGaussian() : AtTPCIonGenerator() {}
   AtTPCIonGeneratorGaussian(const char *name, Int_t z, Int_t a, Int_t q, Int_t mult, Double_t px, Double_t py,
                             Double_t pz, Double_t Ex, Double_t m, Double_t ener, Double_t eLoss = -1)
      : AtTPCIonGenerator(name, z, a, q, mult, px, py, pz, Ex, m, eLoss)
   {
   }

   void SetBeamLimits(Double32_t r = 0, Double32_t z = 0, Double_t theta = 0);

   void SetBeamOrigin(Double32_t x = 0, Double32_t y = 0);

   ClassDefOverride(AtTPCIonGeneratorGaussian, 1);
};

#endif
