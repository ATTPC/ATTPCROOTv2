#include "AtTPCFissionGeneratorV3.h"

#include "AtCSVReader.h"
#include "AtVertexPropagator.h"

#include <FairIon.h>
#include <FairLogger.h>
#include <FairPrimaryGenerator.h>
#include <FairRunSim.h>

#include <TDatabasePDG.h>
#include <TFile.h>
#include <TObject.h> // for TObject
#include <TParticlePDG.h>
#include <TTree.h>
#include <TVirtualMC.h> //For gMC
#include <TVirtualMCStack.h>

#include <iostream>

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
      auto *ion = new FairIon(TString::Format("Ion_%d_%d", Z, A), Z, A, Z);
      FairRunSim::Instance()->AddNewIon(ion);
   }
}

void AtTPCFissionGeneratorV3::loadFissionFragmentTree(TString fissionDistro)
{

   fEventFile = new TFile(fissionDistro, "READ"); // NOLINT (belongs to ROOT)

   if (fEventFile->IsZombie())
      LOG(fatal) << "Could not open file with decay fragments: " << fissionDistro;

   fEventTree = dynamic_cast<TTree *>(fEventFile->Get("fragments"));
   if (!fEventTree)
      LOG(fatal) << "Failed to find the tree fragments";

   fEventTree->SetBranchAddress("decayFragments", &fDecayFrags); // NOLINT
   fEventTree->SetBranchAddress("A", &fA);
   fEventTree->SetBranchAddress("Z", &fZ);

   fNumEvents = fEventTree->GetEntries();
}
// Default constructor
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3() = default;

// Generator that takes in a file that specifies the expected distribution of
// fission particles.
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3(const char *name, TString ionList, TString fissionDistro)
   : fDecayFrags(new std::vector<VecPE>()), fA(new std::vector<Int_t>()), fZ(new std::vector<Int_t>())
{
   loadIonList(ionList);
   loadFissionFragmentTree(fissionDistro);
}

// Deep copy constructor
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3(AtTPCFissionGeneratorV3 &rhs) {}

AtTPCFissionGeneratorV3::~AtTPCFissionGeneratorV3() = default;

Bool_t AtTPCFissionGeneratorV3::ReadEvent(FairPrimaryGenerator *primeGen)
{
   fPrimeGen = primeGen;

   // If this is a beam-like event don't do anything
   if (AtVertexPropagator::Instance()->GetDecayEvtCnt() % 2 == 0) {
      LOG(debug) << "AtTPCFissionGeneratorV3: Skipping beam-like event";
   } else {
      LOG(debug) << "AtTPCFissionGeneratorV3: Runing reaction-like event";
      generateEvent();
   }

   AtVertexPropagator::Instance()->IncDecayEvtCnt();
   return true;
}

void AtTPCFissionGeneratorV3::generateEvent()
{
   setBeamParameters();
   fEventTree->GetEntry(fCurrEvent);

   for (int i = 0; i < fDecayFrags->size(); ++i)
      generateFragment(fDecayFrags->at(i), fA->at(i), fZ->at(i));

   LOG(debug) << "Wrote tracks for fission root event: " << fCurrEvent;
   LOG(debug) << "Wrote tracks for MC event: " << AtVertexPropagator::Instance()->GetDecayEvtCnt();
   fCurrEvent++;
}

VecPE AtTPCFissionGeneratorV3::getBeam4Vec()
{
   Double_t fVEn =
      AtVertexPropagator::Instance()->GetEnergy() + AtVertexPropagator::Instance()->GetBeamMass() * 931.494;
   Double_t fVPx = AtVertexPropagator::Instance()->GetPx() * 1000;
   Double_t fVPy = AtVertexPropagator::Instance()->GetPy() * 1000;
   Double_t fVPz = AtVertexPropagator::Instance()->GetPz() * 1000;
   VecPE beam;
   beam.SetPxPyPzE(fVPx, fVPy, fVPz, fVEn);
   return beam;
}

Cartesian3D AtTPCFissionGeneratorV3::getVertex()
{
   return {AtVertexPropagator::Instance()->GetVx(), AtVertexPropagator::Instance()->GetVy(),
           AtVertexPropagator::Instance()->GetVz()};
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
   AtVertexPropagator::Instance()->SetTrackEnergy(nextTrackID, kinE);
   AtVertexPropagator::Instance()->SetTrackAngle(nextTrackID, angle);

   // Requires GeV
   // NOLINTNEXTLINE
   fPrimeGen->AddTrack(particle->PdgCode(), labP.Px() / 1000, labP.Py() / 1000, labP.Pz() / 1000, fVertex.X(),
                       fVertex.Y(), fVertex.Z());
}

ClassImp(AtTPCFissionGeneratorV3);
