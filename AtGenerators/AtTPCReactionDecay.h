#ifndef AtTPCReactionDecay_H
#define AtTPCReactionDecay_H

#include "AtReactionGenerator.h"

#include <FairIon.h>
#include <FairParticle.h>

#include <Rtypes.h>
#include <TDatabasePDG.h>
#include <TGenPhaseSpace.h>
#include <TLorentzVector.h>
#include <TMath.h>
#include <TParticle.h>
#include <TParticlePDG.h>
#include <TRandom.h>
#include <TString.h>
#include <TVector3.h>

#include <memory>
#include <vector>

class FairPrimaryGenerator;
class TBuffer;
class TClass;
class TMemberInspector;

struct DecayIon {
   std::vector<Int_t> z;
   std::vector<Int_t> a;
   std::vector<Int_t> q;
   std::vector<Double_t> mass;
   Int_t multiplicity;
   Double_t exEnergy;
   Double_t spEnergy;
   TVector3 momentum;
   Int_t trackID;
};

class AtTPCReactionDecay : public AtReactionGenerator {

private:
   DecayIon fScatter;
   DecayIon fRecoil;
   Int_t fZBeam{};
   Int_t fABeam{};
   Double_t fPxBeam;
   Double_t fPyBeam;
   Double_t fPzBeam;
   Bool_t fIsDecay{};
   Double_t fBeamMass{};
   Double_t fTargetMass{};
   Double_t fEnergy{};
   Double_t fVx, fVy, fVz;
   Int_t fNumIons{0};
   Int_t fNumTracks{0};

   std::vector<std::unique_ptr<FairIon>> fIon;
   std::vector<std::unique_ptr<FairParticle>> fParticle;
   std::vector<TString> fPType;

   void RegisterIons(DecayIon &ion);
   std::vector<TLorentzVector> GetDecay(DecayIon &ion);
   bool PropagateIons(std::vector<TLorentzVector>, FairPrimaryGenerator *primGen);

public:
   AtTPCReactionDecay();
   AtTPCReactionDecay(DecayIon &scatter, DecayIon &recoil, Int_t zBeam, Int_t aBeam, Double_t massBeam,
                      Double_t massTarget);
   virtual ~AtTPCReactionDecay() = default;

   virtual Bool_t GenerateReaction(FairPrimaryGenerator *primGen) override;

   ClassDefOverride(AtTPCReactionDecay, 1)
};

#endif
