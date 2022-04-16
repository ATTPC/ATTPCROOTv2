#include "AtTPC20MgDecay.h"

#include <FairLogger.h>
#include <FairPrimaryGenerator.h>

#include <TDatabasePDG.h>
#include <TMath.h>
#include <TParticlePDG.h>
#include <TRandom.h>

#include <cmath>
#include <iostream>
#include <map>

// -----   Default constructor   ------------------------------------------
AtTPC20MgDecay::AtTPC20MgDecay()
   : fOnlyAPBranch(false), fBoxVtxIsSet(false), fX(0), fY(0), fZ(0), fX1(0), fY1(0), fZ1(0), fX2(0), fY2(0), fZ2(0)
{
}

// -----   Destructor   ---------------------------------------------------
AtTPC20MgDecay::~AtTPC20MgDecay() = default;

Bool_t AtTPC20MgDecay::Init()
{
   // Initialize generator
   return kTRUE;
}

// -----   Public method ReadEvent   --------------------------------------
Bool_t AtTPC20MgDecay::ReadEvent(FairPrimaryGenerator *primGen)
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
   // Check for particle type
   TDatabasePDG *pdgBase = TDatabasePDG::Instance();
   TParticlePDG *protonParticle = pdgBase->GetParticle(protonPDGID);
   TParticlePDG *alphaParticle = pdgBase->GetParticle(alphaPDGID);
   if (!protonParticle)
      LOG(fatal) << "AtTPC20MgDecay: PDG code " << protonPDGID << " (proton) not defined.";
   Double32_t protonMass = protonParticle->Mass(); // NOLINT
   if (!alphaParticle)
      LOG(fatal) << "AtTPC20MgDecay: PDG code " << alphaPDGID << " (alpha) not defined.";
   Double32_t alphaMass = alphaParticle->Mass(); // NOLINT

   std::cout << " protonMass: " << protonMass << std::endl;
   std::cout << "alphaMass: " << alphaMass << std::endl;

   // Double32_t kinEneProton = 0.001210;  //GeV
   // Double32_t kinEneAlpha = 0.000506;  //GeV

   Double32_t ptProton = 0, pxProton = 0, pyProton = 0, pzProton = 0;
   Double32_t pabsProton = 0.0469; // GeV/c
   Double32_t thetaProton = acos(gRandom->Uniform(-1, 1));
   Double32_t phiProton = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzProton = pabsProton * TMath::Cos(thetaProton);
   ptProton = pabsProton * TMath::Sin(thetaProton);
   pxProton = ptProton * TMath::Cos(phiProton);
   pyProton = ptProton * TMath::Sin(phiProton);

   Double32_t ptAlpha = 0, pxAlpha = 0, pyAlpha = 0, pzAlpha = 0;
   Double32_t pabsAlpha = 0.06162; // GeV/c
   Double32_t thetaAlpha = acos(gRandom->Uniform(-1, 1));
   Double32_t phiAlpha = gRandom->Uniform(0, 360) * TMath::DegToRad();
   pzAlpha = pabsAlpha * TMath::Cos(thetaAlpha);
   ptAlpha = pabsAlpha * TMath::Sin(thetaAlpha);
   pxAlpha = ptAlpha * TMath::Cos(phiAlpha);
   pyAlpha = ptAlpha * TMath::Sin(phiAlpha);

   primGen->AddTrack(22, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // dummy photon for track ID 0
   primGen->AddTrack(protonPDGID, pxProton, pyProton, pzProton, fX, fY, fZ);
   primGen->AddTrack(alphaPDGID, pxAlpha, pyAlpha, pzAlpha, fX, fY, fZ);

   return kTRUE;
}

ClassImp(AtTPC20MgDecay)
