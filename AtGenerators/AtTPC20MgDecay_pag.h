
#ifndef AtTPC20MGDECAY_pag_H
#define AtTPC20MGDECAY_pag_H

#include <Rtypes.h> // for Double32_t, Bool_t, THashConsistencyHolder

#include "FairGenerator.h"
class TBuffer;
class TClass;
class TMemberInspector;
class FairPrimaryGenerator;

class AtTPC20MgDecay_pag : public FairGenerator {

public:
   /** Default constructor **/
   AtTPC20MgDecay_pag();

   /** Destructor **/
   virtual ~AtTPC20MgDecay_pag() = default;

   /** Initializer **/
   virtual Bool_t Init();

   void SetXYZ(Double32_t x = 0, Double32_t y = 0, Double32_t z = 0)
   {
      fX = x;
      fY = y;
      fZ = z;
   }

   void SetBoxXYZ(Double32_t x1 = 0, Double32_t y1 = 0, Double32_t z1 = 0, Double32_t x2 = 0, Double32_t y2 = 0,
                  Double32_t z2 = 0)
   {
      fX1 = x1;
      fY1 = y1;
      fZ1 = z1;
      fX2 = x2;
      fY2 = y2;
      fZ2 = z2;
      fBoxVtxIsSet = kTRUE;
   }

   virtual Bool_t ReadEvent(FairPrimaryGenerator *primGen);

   void ShowOnlyAlphaProtonBranch() { fOnlyAPBranch = kTRUE; };
   void SetNuclearDecayChain() { fNuclearDecayChainIsSet = kTRUE; };
   void SetDecayChainPoint(Double32_t ParticleEnergy = 0, Double32_t ParticleBranchingRatio = 0);

private:
   Bool_t fOnlyAPBranch; // True if only the beta-alpha-proton branch is visible
   Bool_t fBoxVtxIsSet;  // True if box vertex is set

   Double32_t fX, fY, fZ;                   // Point vertex coordinates [cm]
   Double32_t fX1, fY1, fZ1, fX2, fY2, fZ2; // Box vertex coords (x1,y1,z1)->(x2,y2,z2)
   Bool_t fNuclearDecayChainIsSet;
   Int_t fParticlesDefinedInNuclearDecay;
   Double32_t fParticleEnergies[3] = {0, 0, 0};
   Double32_t fParticleBranchingRatios[3] = {0, 0, 0};
   ClassDef(AtTPC20MgDecay_pag, 1)
};

#endif
