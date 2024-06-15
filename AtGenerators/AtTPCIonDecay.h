#ifndef AtTPCIonDecay_H
#define AtTPCIonDecay_H

#include "AtReactionGenerator.h"

#include <FairGenerator.h>
#include <FairIon.h>
#include <FairParticle.h>

#include <Rtypes.h>
#include <TString.h>

#include <memory>
#include <vector>

class FairPrimaryGenerator;
class TBuffer;
class TClass;
class TMemberInspector;

class AtTPCIonDecay : public AtReactionGenerator {

public:
   /** Default constructor **/
   AtTPCIonDecay();

   /** Default constructor
   ** For the generation of ions with atomic number z and mass number a.
   ** By default, the mass equals a times the proton mass and the
   ** excitation energy is zero. This can be changed with the
   ** respective modifiers.
   **@param z         Atomic number
   **@param a         Atomic mass
   **@param q         Electric charge [e]
   **@param mult      Number of ions per event
   **@param px,py,pz  Momentum components [GeV] per nucleon!
   **@param vx,vy,vz  Vertex coordinates [cm]
   **/
   AtTPCIonDecay(std::vector<std::vector<Int_t>> *z, std::vector<std::vector<Int_t>> *a,
                 std::vector<std::vector<Int_t>> *q, std::vector<std::vector<Double_t>> *mass, Int_t ZB, Int_t AB,
                 Double_t BMass, Double_t TMass, Double_t ExEnergy, std::vector<Double_t> *SepEne);

   AtTPCIonDecay(const AtTPCIonDecay &);

   AtTPCIonDecay &operator=(const AtTPCIonDecay &) { return *this; }

   virtual bool GenerateReaction(FairPrimaryGenerator *primGen) override;

   /** Destructor **/
   virtual ~AtTPCIonDecay() = default;

private:
   static Int_t fgNIon;                                     //! Number of the instance of this class
   std::vector<Int_t> fMult;                                // Multiplicity per decay channel
   Int_t fNbCases;                                          // Number of decay channel
   std::vector<Double_t> fPx, fPy, fPz;                     // Momentum components [GeV] per nucleon
   std::vector<std::vector<Double_t>> fMasses;              // Masses of the N products
   Double_t fVx, fVy, fVz;                                  // Vertex coordinates [cm]
   std::vector<std::vector<std::unique_ptr<FairIon>>> fIon; // Pointer to the FairIon to be generated
   std::vector<std::vector<std::unique_ptr<FairParticle>>> fParticle;
   std::vector<std::vector<TString>> fPType;
   std::vector<Int_t> fQ;       // Electric charge [e]
   Double_t fBeamEnergy{};      // Residual beam energy for phase calculation
   Double_t fBeamEnergy_buff{}; // Residual beam energy for phase calculation
   Int_t fZBeam{};
   Int_t fABeam{};
   Double_t fPxBeam;
   Double_t fPyBeam;
   Double_t fPzBeam;
   Bool_t fIsDecay{};
   Double_t fBeamMass{};
   Double_t fTargetMass{};
   Double_t fExEnergy{};
   std::vector<Double_t> fSepEne;

   ClassDefOverride(AtTPCIonDecay, 5)
};

#endif
