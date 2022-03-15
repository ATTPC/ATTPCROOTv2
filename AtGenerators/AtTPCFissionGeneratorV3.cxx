#include "AtTPCFissionGeneratorV3.h"

#include <fstream>
#include <iostream>

#include "FairIon.h"
#include "FairLogger.h"
#include "FairParticle.h"
#include "FairPrimaryGenerator.h"
#include "FairRunSim.h"

#include "TDatabasePDG.h"
#include "TVirtualMC.h" //For gMC
#include "TFile.h"
#include "TTree.h"

#include "Math/GenVector/LorentzVector.h"
#include "Math/Boost.h"

#include "AtVertexPropagator.h"
#include "AtCSVReader.h"

void AtTPCFissionGeneratorV3::loadIonList(TString ionList)
{
   std::ifstream fileIn(ionList.Data());

   if (!fileIn.is_open())
      LOG(fatal) << "Failed to open file: " << ionList;

   // Loop through each row in the csv and cast each entry to an integer.
   for (auto &row : CSVRange<int>(fileIn)) {
      if (row.size() != 2)
         continue;

      int A = row[0];
      int Z = row[1];
      FairIon *ion = new FairIon(TString::Format("Ion_%d_%d", Z, A), Z, A, Z);
      FairRunSim::Instance()->AddNewIon(ion);
   }
}

void AtTPCFissionGeneratorV3::loadFissionFragmentTree(TString fissionDistro)
{

   fEventFile = new TFile(fissionDistro, "READ");

   if (fEventFile->IsZombie())
      LOG(fatal) << "Could not open file with decay fragments: " << fissionDistro;

   fEventTree = (TTree *)fEventFile->Get("fragments");
   if (!fEventTree)
      LOG(fatal) << "Failed to find the tree fragments";

   fEventTree->SetBranchAddress("decayFragments", &fDecayFrags);
   fEventTree->SetBranchAddress("A", &fA);
   fEventTree->SetBranchAddress("Z", &fZ);

   fNumEvents = fEventTree->GetEntries();
}
// Default constructor
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3() {}

// Generator that takes in a file that specifies the expected distribution of
// fission particles.
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3(const char *name, TString ionList, TString fissionDistro)
   : fDecayFrags(new std::vector<VecPE>()), fA(new std::vector<Int_t>()), fZ(new std::vector<Int_t>()), fCurrEvent(0)
{
   loadIonList(ionList);
   loadFissionFragmentTree(fissionDistro);
}

// Deep copy constructor
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3(AtTPCFissionGeneratorV3 &rhs) {}

AtTPCFissionGeneratorV3::~AtTPCFissionGeneratorV3() {}

Bool_t AtTPCFissionGeneratorV3::ReadEvent(FairPrimaryGenerator *primeGen)
{
   fPrimeGen = primeGen;

   // If this is a beam-like event don't do anything
   if (gAtVP->GetDecayEvtCnt() % 2 == 0) {
      LOG(debug) << "AtTPCFissionGeneratorV3: Skipping beam-like event";
   } else {
      LOG(debug) << "AtTPCFissionGeneratorV3: Runing reaction-like event";
      generateEvent();
   }

   gAtVP->IncDecayEvtCnt();
   return true;
}

void AtTPCFissionGeneratorV3::generateEvent()
{
   setBeamParameters();
   fEventTree->GetEntry(fCurrEvent);

   for (int i = 0; i < fDecayFrags->size(); ++i)
      generateFragment(fDecayFrags->at(i), fA->at(i), fZ->at(i));

   LOG(debug) << "Wrote tracks for fission root event: " << fCurrEvent;
   LOG(debug) << "Wrote tracks for MC event: " << gAtVP->GetDecayEvtCnt();
   fCurrEvent++;
}

VecPE AtTPCFissionGeneratorV3::getBeam4Vec()
{
   Double_t fVEn = gAtVP->GetEnergy() + gAtVP->GetBeamMass() * 931.494;
   Double_t fVPx = gAtVP->GetPx() * 1000;
   Double_t fVPy = gAtVP->GetPy() * 1000;
   Double_t fVPz = gAtVP->GetPz() * 1000;
   VecPE beam;
   beam.SetPxPyPzE(fVPx, fVPy, fVPz, fVEn);
   return beam;
}

Cartesian3D AtTPCFissionGeneratorV3::getVertex()
{
   return Cartesian3D(gAtVP->GetVx(), gAtVP->GetVy(), gAtVP->GetVz());
}

void AtTPCFissionGeneratorV3::setBeamParameters()
{
   fVertex = getVertex();
   fBeamBoost = ROOT::Math::Boost(getBeam4Vec().BoostToCM());
   fBeamBoost.Invert();
}

void AtTPCFissionGeneratorV3::generateFragment(VecPE &P, Int_t A, Int_t Z)
{
   TString particleName = TString::Format("Ion_%d_%d", Z, A);
   auto particle = TDatabasePDG::Instance()->GetParticle(particleName);
   if (!particle)
      LOG(fatal) << "Couldn't find particle " << particleName << " in database!";

   auto labP = fBeamBoost(P);
   auto kinE = labP.E() - labP.mass();
   auto angle = labP.Theta();

   std::cout << std::endl;
   LOG(debug) << TString::Format(
      "AtTPCFissionGeneratorV3: Generating ion of type %s with  CoM momentum (%f, %f, %f) MeV/c", particleName.Data(),
      P.Px(), P.Py(), P.Pz());
   LOG(debug) << TString::Format("Lab momentum (%f, %f, %f) MeV/c at (%f, %f, %f) cm with E = %f and angle = %f",
                                 labP.Px(), labP.Py(), labP.Pz(), fVertex.X(), fVertex.Y(), fVertex.Z(), kinE, angle);

   auto nextTrackID = gMC->GetStack()->GetNtrack();
   gAtVP->SetTrackEnergy(nextTrackID, kinE);
   gAtVP->SetTrackAngle(nextTrackID, angle);

   // Requires GeV
   fPrimeGen->AddTrack(particle->PdgCode(), labP.Px() / 1000, labP.Py() / 1000, labP.Pz() / 1000, fVertex.X(),
                       fVertex.Y(), fVertex.Z());
}

ClassImp(AtTPCFissionGeneratorV3);
