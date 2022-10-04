#include "AtTPCFissionGeneratorV2.h"

#include "AtVertexPropagator.h"

#include <FairIon.h>
#include <FairPrimaryGenerator.h>
#include <FairRunSim.h>

#include <TDatabasePDG.h>
#include <TFile.h>
#include <TObject.h> // for TObject
#include <TParticlePDG.h>
#include <TTree.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <utility>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";

Int_t AtTPCFissionGeneratorV2::fgNIon = 0;

AtTPCFissionGeneratorV2::AtTPCFissionGeneratorV2()
   : fP1x(0.), fP1y(0.), fP1z(0.), fP2x(0.), fP2y(0.), fP2z(0.), fVx(0.), fVy(0.), fVz(0.)
{
   //  cout << "-W- AtTPCIonGenerator: "
   //      << " Please do not use the default constructor! " << endl;
}

// -----   Default constructor   ------------------------------------------
AtTPCFissionGeneratorV2::AtTPCFissionGeneratorV2(const char *name, TString simfile)
   : fP1x(0.), fP1y(0.), fP1z(0.), fP2x(0.), fP2y(0.), fP2z(0.), fVx(0.), fVy(0.), fVz(0.)
{

   TString dir = getenv("VMCWORKDIR");
   TString fFileNamebase;
   std::map<TString, FairIon *> fIonMap; //!

   fFileNamebase = dir + "/macro/Simulation/database/ionlist.txt";
   std::cout << " AtTPCFissionGenerator: Opening input file " << fFileNamebase << std::endl;
   // Open first the file to register all new ions.
   std::ifstream fInputFilebase(fFileNamebase);
   if (!fInputFilebase.is_open())
      Fatal("AtTPCFissionGenerator", "Cannot open input file.");

   std::cout << "AtTPCFissionGenerator: Looking for ions..." << std::endl;

   Int_t nIons = 0;
   Int_t eventId = 1;
   Double_t pBeam, b;

   // Define track variables to be read from file
   Int_t iPid = -1;
   Int_t A, Z, qq = 0;

   fIonMap.clear();

   fInputFilebase >> Z >> A;

   while (!fInputFilebase.eof()) {

      if (fInputFilebase.eof())
         continue;

      char buffer[40];
      sprintf(buffer, "Ion_%d_%d", A, Z);
      TString ionName(buffer);

      auto *ion = new FairIon(ionName, Z, A, qq); // NOLINT (I think the run takes ownership of this memory?)
      fIonMap[ionName] = ion;
      nIons++;

      fInputFilebase >> Z >> A;

   } //!

   FairRunSim *run = FairRunSim::Instance();
   std::map<TString, FairIon *>::iterator mapIt;
   for (mapIt = fIonMap.begin(); mapIt != fIonMap.end(); mapIt++) {
      FairIon *ion = (*mapIt).second;
      run->AddNewIon(ion);
   }

   std::cout << cYELLOW << "AtTPCFissionGenerator: " << nIons << " ions registered." << cNORMAL << std::endl;

   TString simfilepath = dir + "/macro/Simulation/data/" + simfile;
   auto *f = new TFile(simfilepath.Data()); // NOLINT (belongs to ROOT)
   if (f->IsZombie()) {
      std::cout << cRED << " AtTPCFissionGenerator: No simulation file found! Check VMCWORKDIR variable. Exiting... "
                << cNORMAL << std::endl;
      return;
   } else
      std::cout << cGREEN << " AtTPCFissionGenerator : Prototype geometry found in : " << simfilepath.Data() << cNORMAL
                << std::endl;

   auto *fTree = dynamic_cast<TTree *>(f->Get("tree101"));
   Int_t nEvents = fTree->GetEntriesFast();
   std::cout << " Number of events : " << nEvents << std::endl;
   fTree->SetBranchAddress("Evnt", &Evnt);
   fTree->SetBranchAddress("Ntrack", &Ntrack);
   fTree->SetBranchAddress("Aout", Aout);
   fTree->SetBranchAddress("Zout", Zout);
   fTree->SetBranchAddress("fOutPx", fOutPx);
   fTree->SetBranchAddress("fOutPy", fOutPy);
   fTree->SetBranchAddress("fOutPz", fOutPz);
   pTree.push_back(fTree);

   event = 0;
}

// -----   Public method ReadEvent   --------------------------------------
Bool_t AtTPCFissionGeneratorV2::ReadEvent(FairPrimaryGenerator *primGen)
{

   fVx = 0., fVy = 0., fVz = 0.;
   Double_t uma = 931.494028, mp = 938.272013, c = 29.972458;
   Double_t px, py, pz;

   TDatabasePDG *fPDG = TDatabasePDG::Instance();

   pTree.at(0)->GetEntry(event);

   // fIsDecay = kFALSE;

   // fBeamEnergy = gAtVP->GetEnergy();
   // std::cout<<" -I- AtTPC2Body Residual energy  : "<<gAtVP->GetEnergy()<<std::endl;

   // fPxBeam = gAtVP->GetPx();
   // fPyBeam = gAtVP->GetPy();
   // fPzBeam = gAtVP->GetPz();

   // gAtVP->SetRecoilE(Ene.at(1));
   // gAtVP->SetRecoilA(Ang.at(1)*180.0/TMath::Pi());
   // gAtVP->SetScatterE(Ene.at(0));
   // gAtVP->SetScatterA(Ang.at(0)*180.0/TMath::Pi());

   // Propagate the vertex of the previous event

   // fVx = gAtVP->GetVx();
   // fVy = gAtVP->GetVy();
   // fVz = gAtVP->GetVz();

   // if(i>1 && gAtVP->GetDecayEvtCnt() && pdgType!=1000500500 && fPType.at(i)=="Ion" ){// TODO: Dirty way to propagate
   // only the products (0 and 1 are beam and target respectively)

   // Define event
   Int_t I = 0;

   // Define track variables
   Int_t iPid = -1;
   Int_t ia1 = 12;
   Int_t iz1 = 6;
   Int_t pdgType = 0;

   for (Int_t j = 0; j < Ntrack; j++) {

      ia1 = Aout[j];
      iz1 = Zout[j];

      if (ia1 > 2 && iz1 > 2) {
         if (iPid < 0) {
            char ionName[40];
            sprintf(ionName, "Ion_%d_%d", ia1, iz1);
            TParticlePDG *part = fPDG->GetParticle(ionName);
            if (!part) {
               std::cout << "AtTPCFissionGenerator::ReadEvent: Cannot find " << ionName << " in database!" << std::endl;
               // continue;
            }
            if (part)
               pdgType = part->PdgCode();
         } else
            pdgType = ia1; // "normal" particle

         px = fOutPx[j];
         py = fOutPy[j];
         pz = fOutPz[j];

         primGen->AddTrack(pdgType, px, py, pz, fVx, fVy, fVz);
      }
   }

   AtVertexPropagator::Instance()
      ->IncDecayEvtCnt(); // TODO: Okay someone should put a more suitable name but we are on a hurry...
   std::cout << cRED << " Fission event : " << event << cNORMAL << std::endl;
   event++;

   return kTRUE;
}

ClassImp(AtTPCFissionGeneratorV2)
