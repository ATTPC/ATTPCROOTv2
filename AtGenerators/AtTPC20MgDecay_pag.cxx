

#include "AtTPC20MgDecay_pag.h"

#include <FairLogger.h> // for Logger, LOG

#include "FairPrimaryGenerator.h"
#include "TDatabasePDG.h"
#include "TMath.h"
#include "TParticlePDG.h"
#include "TRandom.h"

#include <cmath>    // for acos
#include <iostream> // for operator<<, endl, basic_ostream
#include <map>      // for allocator

// -----   Default constructor   ------------------------------------------
AtTPC20MgDecay_pag::AtTPC20MgDecay_pag()
   : fOnlyAPBranch(false), fBoxVtxIsSet(false), fNuclearDecayChainIsSet(false), fParticlesDefinedInNuclearDecay(0),
     fX(0), fY(0), fZ(0), fX1(0), fY1(0), fZ1(0), fX2(0), fY2(0), fZ2(0)
{
}

Bool_t AtTPC20MgDecay_pag::Init()
{
   // Initialize generator
   return true;
}

// -----   Public method ReadEvent   --------------------------------------
Bool_t AtTPC20MgDecay_pag::ReadEvent(FairPrimaryGenerator *primGen)
{

   if (fBoxVtxIsSet) {
      fX = gRandom->Uniform(fX1, fX2);
      fY = gRandom->Uniform(fY1, fY2);
      fZ = gRandom->Uniform(fZ1, fZ2);
   }

   // Bool_t

   if (!fOnlyAPBranch) {
      // all gammas!!!! calculating branching ratios and prob of emission
   }

   // Proton of 1210keV and alpha of 506keV
   Int_t protonPDGID = 2212;
   Int_t alphaPDGID = 1000020040;
   Int_t gammaPDGID = 22;
   Int_t betaPDGID = 11; // NOLINT
   // Check for particle type
   TDatabasePDG *pdgBase = TDatabasePDG::Instance();
   TParticlePDG *protonParticle = pdgBase->GetParticle(protonPDGID);
   TParticlePDG *alphaParticle = pdgBase->GetParticle(alphaPDGID);
   TParticlePDG *gammaParticle = pdgBase->GetParticle(gammaPDGID);
   TParticlePDG *betaParticle = pdgBase->GetParticle(betaPDGID); // NOLINT
   if (!protonParticle)
      LOG(fatal) << "AtTPC20MgDecay_pag: PDG code " << protonPDGID << " (proton) not defined.";
   Double32_t protonMass = protonParticle->Mass(); // NOLINT
   if (!gammaParticle)
      LOG(fatal) << "AtTPC20MgDecay_pag: PDG code" << gammaPDGID << " (gamma) not defined.";
   Double32_t gammaMass = gammaParticle->Mass(); // NOLINT
   if (!alphaParticle)
      LOG(fatal) << "AtTPC20MgDecay_pag: PDG code " << alphaPDGID << " (alpha) not defined.";
   Double32_t alphaMass = alphaParticle->Mass(); // NOLINT

   std::cout << " protonMass: " << protonMass << std::endl;
   std::cout << " gammaMass: " << gammaMass << std::endl;
   std::cout << "alphaMass: " << alphaMass << std::endl;

   Double32_t ptProton = 0, pxProton = 0, pyProton = 0, pzProton = 0;
   // Double32_t pabsProton = 0.0470; // GeV/c
   Double32_t pabsProton = 0.0763; // GeV/c
   Double32_t thetaProton = acos(gRandom->Uniform(-1, 1));
   Double32_t brp = 0;
   Double32_t phiProton = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton = pabsProton * TMath::Cos(thetaProton);
   ptProton = pabsProton * TMath::Sin(thetaProton);
   pxProton = ptProton * TMath::Cos(phiProton);
   pyProton = ptProton * TMath::Sin(phiProton);

   Double32_t ptAlpha = 0, pxAlpha = 0, pyAlpha = 0, pzAlpha = 0;
   Double32_t bra = 0;
   Double32_t pabsAlpha = 0.06162; // GeV/c
   Double32_t thetaAlpha = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha = pabsAlpha * TMath::Cos(thetaAlpha);
   ptAlpha = pabsAlpha * TMath::Sin(thetaAlpha);
   pxAlpha = ptAlpha * TMath::Cos(phiAlpha);
   pyAlpha = ptAlpha * TMath::Sin(phiAlpha);

   Double32_t ptGamma = 0, pxGamma = 0, pyGamma = 0, pzGamma = 0; // NOLINT
   Double32_t pabsGamma = 0.004033;                               // GeV/c
   // Double32_t brg=0;
   Double32_t thetaGamma = acos(gRandom->Uniform(0, 1));
   Double32_t phiGamma = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzGamma = pabsGamma * TMath::Cos(thetaGamma); // NOLINT
   ptGamma = pabsGamma * TMath::Sin(thetaGamma);
   pxGamma = ptGamma * TMath::Cos(phiGamma); // NOLINT
   pyGamma = ptGamma * TMath::Sin(phiGamma); // NOLINT

   if (fNuclearDecayChainIsSet) {

      if (protonPDGID != 2212)
         LOG(fatal) << "AtTPC20MgDecay_pagGenerator:PDG code" << protonPDGID << "is not a proton!";
      // if(protonPDGID == 2212)
      brp = gRandom->Uniform(0, 1);
      bra = gRandom->Uniform(0, 1);
      // brb =gRandom->Uniform(0,1);
      for (Int_t i = 0; i < fParticlesDefinedInNuclearDecay; i++) {

         if (brp <= 1) {
            {
               Double32_t ProtonMomentum = TMath::Sqrt(pxProton * pxProton + pyProton * pyProton + pzProton * pzProton);
               pxProton = pxProton * fParticleEnergies[i] / ProtonMomentum;
               pyProton = pyProton * fParticleEnergies[i] / ProtonMomentum;
               pzProton = pzProton * fParticleEnergies[i] / ProtonMomentum;
            }

            // if(bra<=0.716){
            if (bra <= 1) {
               Double32_t AlphaMomentum = TMath::Sqrt(pxAlpha * pxAlpha + pyAlpha * pyAlpha + pzAlpha * pzAlpha);
               pxAlpha = pxAlpha * fParticleEnergies[i] / AlphaMomentum;
               pyAlpha = pyAlpha * fParticleEnergies[i] / AlphaMomentum;
               pzAlpha = pzAlpha * fParticleEnergies[i] / AlphaMomentum;
            }
         }
      }

      primGen->AddTrack(22, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // dummy photon for track ID 0
      if (brp <= 1) {
         {
            primGen->AddTrack(protonPDGID, pxProton, pyProton, pzProton, fX, fY, fZ);
         }
         if (bra <= 1) {
            // if(bra1<=1){
            primGen->AddTrack(alphaPDGID, pxAlpha, pyAlpha, pzAlpha, fX, fY, fZ);
         }
      }
   }

   return kTRUE;
}
void AtTPC20MgDecay_pag::SetDecayChainPoint(Double32_t ParticleEnergy, Double32_t ParticleBranchingRatio)
{

   for (Int_t i = 0; i < fParticlesDefinedInNuclearDecay; i++) {
      fParticleEnergies[i] = ParticleEnergy;
      fParticleBranchingRatios[i] = ParticleBranchingRatio;
   }
}

ClassImp(AtTPC20MgDecay_pag)
