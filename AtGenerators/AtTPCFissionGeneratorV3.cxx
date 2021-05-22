#include "AtTPCFissionGeneratorV3.h"
#include "TFile.h"
#include <set>

// Default constructor
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3() : nTracks(0) {}

// Generator that takes in a file that specifies the expected distribution of
// fission particles.
AtTPCFissionGeneratorV3::AtTPCFissionGeneratorV3(const char *name, TString ionList, TString fissionDistro) : nTracks(0)
{

   // Get the working directory
   TString workingDir = std::getenv("VCMWORKDIR");

   // Look for the file defining the fission mass distrobution
   std::ifstream fileIn(ionList.Data());

   if (!fileIn.is_open())
      Fatal("AtTPCFissionGeneratorV3", Form("Cannot open input file: %s", ionList.Data()));

   // Read the file until all of the ions have been generated
   std::set<std::vector<int>> ionSet;

   while (!fileIn.eof()) {

      int A = 0, Z = 0;
      fileIn >> Z >> A;

      // If this wasn't a valid ion, then look to make sure we've hit the end of the ion section
      if (A == 0 && Z == 0) {
         char strIn[100];
         fileIn.getline(strIn, 100);
         std::cout << "Invalid ion (A,Z): (" << A << ", " << Z << ") "
                   << " next line: " << strIn << std::endl;
         break;
      }

      ionSet.emplace(std::vector<int>({Z, A}));
      std::cout << "Found ion: " << Z << " " << A << std::endl;

      // Add the ion to the set of found ions
   } // end loop through ion list

   // Get the run instance and register the ions
   auto run = FairRunSim::Instance();
   for (auto it : ionSet) {
      FairIon *ion = new FairIon(TString::Format("Ion_%d_%d", it.at(0), it.at(1)), it.at(0), it.at(1), 0);
      run->AddNewIon(ion);
   }

   // Load the tree
   TFile *treeFile = new TFile(fissionDistro, "READ");
   if (treeFile->IsZombie())
      std::cout << "No file of fission events found!" << std::endl;

   fissionEvents = (TTree *)treeFile->Get("trEvents");
   fissionEvents->SetBranchAddress("nTracks", &nTracks);
   fissionEvents->SetBranchAddress("Aout", Aout);
   fissionEvents->SetBranchAddress("Zout", Zout);
   fissionEvents->SetBranchAddress("pX", pX);
   fissionEvents->SetBranchAddress("pY", pY);
   fissionEvents->SetBranchAddress("pZ", pZ);
   fissionEvents->SetBranchAddress("pT", pT);

   nEvents = fissionEvents->GetEntries();
   event = 0;
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

   fissionEvents->GetEntry(event);

   for (int i = 0; i < nTracks; ++i) {
      // Create the particle
      Int_t pdgType = 0;
      TString partName = TString::Format("Ion_%d_%d", Zout[i], Aout[i]);
      auto part = fPDG->GetParticle(partName);

      if (!part)
         std::cout << "Couldn't find particle " << partName << " in database!" << std::endl;
      else
         pdgType = part->PdgCode();

      auto px = pX[i] / 1000; // Change to GeV/c
      auto py = pY[i] / 1000;
      auto pz = pZ[i] / 1000;
      auto pt = pT[i] / 1000;
      TLorentzVector frag(px, py, pz, pt);
      frag.Boost(boostVec);

      std::cout << std::endl;
      std::cout << "AtTPCFissionGeneratorV3: Generating ion of type " << partName << " with CoM momentum (" << px
                << ", " << py << ", " << pz << ") GeV/c at vertex (" << fVx << ", " << fVy << ", " << fVz << ") cm."
                << std::endl;
      std::cout << "AtTPCFissionGeneratorV3: Generating ion of type " << partName << " with lab momentum (" << frag.Px()
                << ", " << frag.Py() << ", " << frag.Pz() << ") GeV/c at vertex (" << fVx << ", " << fVy << ", " << fVz
                << ") cm." << std::endl
                << std::endl;

      // Requires GeV
      primeGen->AddTrack(pdgType, frag.Px(), frag.Py(), frag.Pz(), fVx, fVy, fVz);

   } // End loop over tracks

   std::cout << "Wrote tracks for fission root event: " << event << std::endl;
   std::cout << "Wrote tracks for MC event: " << gAtVP->GetDecayEvtCnt() << std::endl;
   event++;

   gAtVP->IncDecayEvtCnt();
   return true;
}

ClassImp(AtTPCFissionGeneratorV3);
