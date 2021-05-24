#include "AtTPCFissionGeneratorV3.h"
#include "TFile.h"
#include "AtCSVReader.h"

// Default constructor
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3() {}

// Generator that takes in a file that specifies the expected distribution of
// fission particles.
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3(const char *name, TString ionList, TString fissionDistro)
   : fDecayFrags(new std::vector<VecXYZE>()), fA(new std::vector<Int_t>()), fZ(new std::vector<Int_t>())
{
   // Look for the file defining the fission mass distrobution
   std::ifstream fileIn(ionList.Data());

   if (!fileIn.is_open())
      LOG(fatal) << "Failed to open file: " << ionList;

   // Read the file until all of the ions have been generated
   auto run = FairRunSim::Instance();

   std::cout << "Looping through... " << std::endl;
   for (auto &row : CSVRange<int>(fileIn)) {
      if (row.size() != 2)
         continue;

      int A = row[0];
      int Z = row[1];

      std::cout << "Creating ion: " << Z << A << std::endl;

      FairIon *ion = new FairIon(TString::Format("Ion_%d_%d", Z, A), Z, A, Z);
      run->AddNewIon(ion);

   } // end loop through ion list

   // Load the tree
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
   fCurrEvent = 0;
}

// Deep copy constructor
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3(AtTPCFissionGeneratorV3 &rhs) {}

AtTPCFissionGeneratorV3::~AtTPCFissionGeneratorV3() {}

Bool_t AtTPCFissionGeneratorV3::ReadEvent(FairPrimaryGenerator *primeGen)
{
   // If this is a beam-like event don't do anything
   if (gAtVP->GetDecayEvtCnt() % 2 == 0) {
      std::cout << "AtTPCFissionGeneratorV3: Skipping beam-like event" << std::endl;
      gAtVP->IncDecayEvtCnt();
      return true;
   } else
      std::cout << "AtTPCFissionGeneratorV3: Runing reaction-like event" << std::endl;

   auto fPDG = TDatabasePDG::Instance();
   auto stack = (AtStack *)gMC->GetStack();

   // Read this event and get the current vertex
   Double_t fVx = gAtVP->GetVx();
   Double_t fVy = gAtVP->GetVy();
   Double_t fVz = gAtVP->GetVz();

   // Get the energy and momentum of the beam in MeV and MeV/c
   Double_t fVEn = gAtVP->GetEnergy() + gAtVP->GetBeamMass() * 931.494;
   Double_t fVPx = gAtVP->GetPx() * 1000;
   Double_t fVPy = gAtVP->GetPy() * 1000;
   Double_t fVPz = gAtVP->GetPz() * 1000;

   TLorentzVector beam;
   beam.SetPxPyPzE(fVPx, fVPy, fVPz, fVEn);

   // Get the vector to boost back to the lab frame
   auto boostVec = beam.BoostVector();

   fEventTree->GetEntry(fCurrEvent);
   std::cout << fDecayFrags << std::endl;

   for (int i = 0; i < fDecayFrags->size(); ++i) {
      // Create the particle
      Int_t pdgType = 0;
      TString partName = TString::Format("Ion_%d_%d", fZ->at(i), fA->at(i));
      auto part = fPDG->GetParticle(partName);

      if (!part)
         std::cout << "Couldn't find particle " << partName << " in database!" << std::endl;
      else
         pdgType = part->PdgCode();

      auto frag = fDecayFrags->at(i);

      std::cout << std::endl;
      std::cout << "AtTPCFissionGeneratorV3: Generating ion of type " << partName << " with CoM momentum (" << frag.Px()
                << ", " << frag.Py() << ", " << frag.Pz() << ") MeV/c at vertex (" << fVx << ", " << fVy << ", " << fVz
                << ") cm." << std::endl;
      // std::cout << "AtTPCFissionGeneratorV3: Generating ion of type " << partName << " with lab momentum (" <<
      // frag.Px()
      //        << ", " << frag.Py() << ", " << frag.Pz() << ") GeV/c at vertex (" << fVx << ", " << fVy << ", " << fVz
      //        << ") cm." << std::endl
      //        << std::endl;

      // Requires GeV
      primeGen->AddTrack(pdgType, frag.Px() / 1000, frag.Py() / 1000, frag.Pz() / 1000, fVx, fVy, fVz);

   } // End loop over tracks

   std::cout << "Wrote tracks for fission root event: " << fCurrEvent << std::endl;
   std::cout << "Wrote tracks for MC event: " << gAtVP->GetDecayEvtCnt() << std::endl;
   fCurrEvent++;

   gAtVP->IncDecayEvtCnt();
   return true;
}

ClassImp(AtTPCFissionGeneratorV3);
