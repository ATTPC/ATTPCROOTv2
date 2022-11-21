// -------------------------------------------------------------------------
// -----               AtTPCXSReader implementation file               -----
// -----                Created 03/07/18  by H. Alvarez                -----
// -------------------------------------------------------------------------
#include "AtTPCXSReader.h"

#include "AtVertexPropagator.h"

#include <FairIon.h>
#include <FairParticle.h>
#include <FairPrimaryGenerator.h>
#include <FairRunSim.h>

#include <TDatabasePDG.h>
#include <TH2.h>
#include <TMath.h>
#include <TParticle.h>
#include <TParticlePDG.h>
#include <TRandom.h>
#include <TVector3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

using std::cout;
using std::endl;

Int_t AtTPCXSReader::fgNIon = 0;

AtTPCXSReader::AtTPCXSReader() : fMult(0), fPx(0.), fPy(0.), fPz(0.), fVx(0.), fVy(0.), fVz(0.), fIon(0), fQ(0)
{
   //  cout << "-W- AtTPCXSReader: "
   //      << " Please do not use the default constructor! " << endl;
}

AtTPCXSReader::AtTPCXSReader(const char *name, std::vector<Int_t> *z, std::vector<Int_t> *a, std::vector<Int_t> *q,
                             Int_t mult, std::vector<Double_t> *px, std::vector<Double_t> *py,
                             std::vector<Double_t> *pz, std::vector<Double_t> *mass)
   : fMult(mult), fPx(0.), fPy(0.), fPz(0.), fVx(0.), fVy(0.), fVz(0.), fIon(0), fPType(0.), fQ(0)
{

   fgNIon++;

   fIon.reserve(fMult);

   SetXSFileName();

   TString dir = getenv("VMCWORKDIR");
   TString XSFileName = dir + "/AtGenerators/" + fXSFileName;
   std::cout << " AtTPCXSReader: Opening input file " << XSFileName << std::endl;
   auto fInputXSFile = std::ifstream(XSFileName);
   // std::ifstream*  fInputXSFile = new
   // std::ifstream("/home/ayyadlim/fair_install/AtTPCROOTv2_HAP/AtGenerators/xs_22Mgp_fusionEvaporation.txt");
   if (!fInputXSFile.is_open())
      Fatal("AtTPCXSReader", "Cannot open input file.");

   std::cout << "AtTPCXSReader: opening PACE cross sections..." << std::endl;

   Double_t ene[31];
   Double_t xs[31][18];

   // fixed format
   for (Int_t energies = 0; energies < 31; energies++) {
      fInputXSFile >> ene[energies];
      // std::cout << ene[energies] << " ";
      for (Int_t xsvalues = 0; xsvalues < 18; xsvalues++) {
         fInputXSFile >> xs[energies][xsvalues];
         // std::cout << xs[energies][xsvalues]<< " ";
      }
      // std::cout << std::endl;
   }

   fh_pdf = new TH2F("pdf", "pdf", 31, 0, 31, 18, 0, 180); // NOLINT (ROOT will cleanup)
   for (Int_t energies = 0; energies < 31; energies++) {
      for (Int_t xsvalues = 0; xsvalues < 18; xsvalues++) {
         fh_pdf->SetBinContent(energies + 1, xsvalues + 1, xs[energies][xsvalues]);
      }
   }
   // fh_pdf->Write();

   auto *kProton = new TParticle(); // NOLINT Probably actually a problem though
   kProton->SetPdgCode(2212);

   auto *kNeutron = new TParticle(); // NOLINT Probably actually a problem though
   kNeutron->SetPdgCode(2112);

   char buffer[30];
   for (Int_t i = 0; i < fMult; i++) {
      fPx.push_back(Double_t(a->at(i)) * px->at(i));
      fPy.push_back(Double_t(a->at(i)) * py->at(i));
      fPz.push_back(Double_t(a->at(i)) * pz->at(i));
      Masses.push_back(mass->at(i) * 1000.0);
      fWm.push_back(mass->at(i) * 1000.0);
      FairIon *IonBuff;
      FairParticle *ParticleBuff;
      sprintf(buffer, "Product_Ion%d", i);

      if (a->at(i) != 1) {
         IonBuff = new FairIon(buffer, z->at(i), a->at(i), q->at(i), 0.0, // NOLINT Probably actually a problem though
                               mass->at(i));
         ParticleBuff = // NOLINT Probably actually a problem though
            new FairParticle("dummyPart", 1, 1, 1.0, 0, 0.0, 0.0);
         fPType.emplace_back("Ion");
         std::cout << " Adding : " << buffer << std::endl;
      } else if (a->at(i) == 1 && z->at(i) == 1) {
         IonBuff = new FairIon("dummyIon", 50, 50, 0, 0.0, 100); // NOLINT Probably actually a problem though
         ParticleBuff = new FairParticle(2212, kProton);         // NOLINT Probably actually a problem though
         fPType.emplace_back("Proton");
      } else if (a->at(i) == 1 && z->at(i) == 0) {
         IonBuff = new FairIon("dummyIon", 50, 50, 0, 0.0, 100); // NOLINT Probably actually a problem though
         ParticleBuff = new FairParticle(2112, kNeutron);        // NOLINT Probably actually a problem though
         fPType.emplace_back("Neutron");
      }

      std::cout << " Z " << z->at(i) << " A " << a->at(i) << std::endl;
      // std::cout<<buffer<<std::endl;
      fIon.push_back(IonBuff);
      fParticle.push_back(ParticleBuff);
   }

   FairRunSim *run = FairRunSim::Instance();
   if (!run) {
      std::cout << "-E- FairIonGenerator: No FairRun instantised!" << std::endl;
      Fatal("FairIonGenerator", "No FairRun instantised!");
   }

   for (Int_t i = 0; i < fMult; i++) {
      if (fPType.at(i) == "Ion") {
         std::cout << " In position " << i << " adding an : " << fPType.at(i) << std::endl;
         run->AddNewIon(fIon.at(i));
         std::cout << " fIon name :" << fIon.at(i)->GetName() << std::endl;
         std::cout << " fParticle name :" << fParticle.at(i)->GetName() << std::endl;
      } else if (fPType.at(i) == "Proton") {
         std::cout << " In position " << i << " adding an : " << fPType.at(i) << std::endl;
         // run->AddNewParticle(fParticle.at(i));
         std::cout << " fIon name :" << fIon.at(i)->GetName() << std::endl;
         std::cout << " fParticle name :" << fParticle.at(i)->GetName() << std::endl;
         std::cout << fParticle.at(i)->GetName() << std::endl;
      } else if (fPType.at(i) == "Neutron") {
         std::cout << " In position " << i << " adding an : " << fPType.at(i) << std::endl;
         // run->AddNewParticle(fParticle.at(i));
         std::cout << " fIon name :" << fIon.at(i)->GetName() << std::endl;
         std::cout << " fParticle name :" << fParticle.at(i)->GetName() << std::endl;
         std::cout << fParticle.at(i)->GetName() << std::endl;
      }
   }
}

Bool_t AtTPCXSReader::ReadEvent(FairPrimaryGenerator *primGen)
{
   const Double_t rad2deg = 0.0174532925;

   std::vector<Double_t> Ang; // Lab Angle of the products
   std::vector<Double_t> Ene; // Lab Energy of the products
   Ang.reserve(2);
   Ene.reserve(2);
   fPx.clear();
   fPy.clear();
   fPx.clear();
   fPx.resize(fMult);
   fPy.resize(fMult);
   fPx.resize(fMult);

   fBeamEnergy = AtVertexPropagator::Instance()->GetEnergy();

   // Requires a non zero vertex energy and pre-generated Beam event (not punch thorugh)
   if (AtVertexPropagator::Instance()->GetEnergy() > 0 && AtVertexPropagator::Instance()->GetDecayEvtCnt() % 2 != 0) {
      // proton parameters come from the XS PDF
      Double_t energyFromPDF, thetaFromPDF;
      fh_pdf->GetRandom2(energyFromPDF, thetaFromPDF);

      Ang.push_back(thetaFromPDF * TMath::Pi() / 180); // set angle PROTON (in rad)
      Ene.push_back(energyFromPDF);                    // set energy PROTON

      fPxBeam = AtVertexPropagator::Instance()->GetPx();
      fPyBeam = AtVertexPropagator::Instance()->GetPy();
      fPzBeam = AtVertexPropagator::Instance()->GetPz();

      // Double_t eb = fBeamEnergy + fWm.at(0); // total (beam) projectile energy = projectile kinetic e + mass
      Double_t pb2 = fBeamEnergy * fBeamEnergy + 2.0 * fBeamEnergy * fWm.at(0); //(beam) projectile momentum squared
      // Double_t pb = TMath::Sqrt(pb2);                                           //(beam)projectile momentum
      //  Double_t beta = pb/(eb+fWm.at(1));         // ??check beta of the projectile+target compound check??
      //  Double_t gamma = 1.0/sqrt(1.0-beta*beta);
      Double_t e = fBeamEnergy + fWm.at(0) + fWm.at(1); // total energy (beam+target)
      Double_t e_cm2 = e * e - pb2;                     // cm energy (beam+target) squared
      Double_t e_cm = TMath::Sqrt(e_cm2);               // cm energy (beam+target)
      Double_t t_cm = e_cm - fWm.at(2) - fWm.at(3);     // kinetic energy available for final products

      // HERE REMAINS THE CALCULAtION OF THE ANGLE OF THE SCAtTER AS A FUNCTION OF THE
      // ANGLE AND KINETIC ENERGY OF THE RECOIL (proton)
      // Double_t p_c
      //       tan theta_scatter = p_1*cos(theta1)*sin(theta1)/(p_A - p_1(cos(theta1))*cos(theta1));
      //  with p_1(cos(theta1))=mA*E_1

      Double_t t_scatter = 0;
      if (t_cm - energyFromPDF > 0)
         t_scatter = t_cm - energyFromPDF;
      else
         std::cout << "Kinetic Energy of scatter particle negative!" << std::endl;

      Double_t theta_scatter = 0.05; // TMath::Atan(*TMath::Sin(angleFromPDF)/)   (in rad)

      Ang.push_back(theta_scatter); // set angle ION   DUMMY FOR THE MOMENT!!!!!!!!!!!!!!! CHECK AND SOLVE
      Ene.push_back(t_scatter);     // set energy ION

      AtVertexPropagator::Instance()->SetTrackEnergy(0, Ene.at(0));
      AtVertexPropagator::Instance()->SetTrackAngle(0, Ang.at(0) * 180.0 / TMath::Pi());

      AtVertexPropagator::Instance()->SetTrackEnergy(1, Ene.at(1));
      AtVertexPropagator::Instance()->SetTrackAngle(1, Ang.at(1) * 180.0 / TMath::Pi());

      fPx.at(0) = 0.0;
      fPy.at(0) = 0.0;
      fPz.at(0) = 0.0;
      fPx.at(1) = 0.0;
      fPy.at(1) = 0.0;
      fPz.at(1) = 0.0;

      Double_t phi1 = 0., phi2 = 0.;
      phi1 = 2 * TMath::Pi() * gRandom->Uniform(); // flat probability in phi
      phi2 = phi1 + TMath::Pi();

      // To MeV for Euler Transformation
      TVector3 BeamPos(AtVertexPropagator::Instance()->GetPx() * 1000, AtVertexPropagator::Instance()->GetPy() * 1000,
                       AtVertexPropagator::Instance()->GetPz() * 1000);

      TVector3 direction1 = TVector3(sin(Ang.at(0)) * cos(phi1), sin(Ang.at(0)) * sin(phi1),
                                     cos(Ang.at(0))); // recoil

      TVector3 direction2 = TVector3(sin(Ang.at(1)) * cos(phi2), sin(Ang.at(1)) * sin(phi2),
                                     cos(Ang.at(1))); // scatter

      Double_t p2_recoil = Ene.at(0) * Ene.at(0) + 2.0 * Ene.at(0) * fWm.at(3);
      Double_t p2_scatter = Ene.at(1) * Ene.at(1) + 2.0 * Ene.at(1) * fWm.at(2);
      if (p2_recoil < 0 || p2_scatter < 0)
         std::cout << "Particle momentum negative!" << std::endl;

      Double_t p_recoil = TMath::Sqrt(p2_recoil);
      Double_t p_scatter = TMath::Sqrt(p2_scatter);

      fPx.at(2) = p_scatter * direction2.X() / 1000.0; // To GeV for FairRoot
      fPy.at(2) = p_scatter * direction2.Y() / 1000.0;
      fPz.at(2) = p_scatter * direction2.Z() / 1000.0;

      fPx.at(3) = p_recoil * direction1.X() / 1000.0;
      fPy.at(3) = p_recoil * direction1.Y() / 1000.0;
      fPz.at(3) = p_recoil * direction1.Z() / 1000.0;

      // Particle transport begins here
      for (Int_t i = 0; i < fMult; i++) {
         TParticlePDG *thisPart;
         if (fPType.at(i) == "Ion")
            thisPart = TDatabasePDG::Instance()->GetParticle(fIon.at(i)->GetName());
         else if (fPType.at(i) == "Proton")
            thisPart = TDatabasePDG::Instance()->GetParticle(fParticle.at(i)->GetName());
         else if (fPType.at(i) == "Neutron")
            thisPart = TDatabasePDG::Instance()->GetParticle(fParticle.at(i)->GetName());

         if (!thisPart) {
            if (fPType.at(i) == "Ion")
               std::cout << "-W- FairIonGenerator: Ion " << fIon.at(i)->GetName() << " not found in database!"
                         << std::endl;
            else if (fPType.at(i) == "Proton")
               std::cout << "-W- FairIonGenerator: Particle " << fParticle.at(i)->GetName() << " not found in database!"
                         << std::endl;
            else if (fPType.at(i) == "Neutron")
               std::cout << "-W- FairIonGenerator: Particle " << fParticle.at(i)->GetName() << " not found in database!"
                         << std::endl;
            return kFALSE;
         }

         int pdgType = thisPart->PdgCode();

         // Propagate the vertex of the previous event
         fVx = AtVertexPropagator::Instance()->GetVx();
         fVy = AtVertexPropagator::Instance()->GetVy();
         fVz = AtVertexPropagator::Instance()->GetVz();

         // TODO: Dirty way to propagate only the products (0 and 1 are beam and target respectively)
         if (i > 1 && AtVertexPropagator::Instance()->GetDecayEvtCnt() && pdgType != 1000500500 &&
             fPType.at(i) == "Ion") {
            std::cout << "-I- FairIonGenerator: Generating ions of type " << fIon.at(i)->GetName() << " (PDG code "
                      << pdgType << ")" << std::endl;
            std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
                      << ") Gev from vertex (" << fVx << ", " << fVy << ", " << fVz << ") cm" << std::endl;
            primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);
         } else if (i > 1 && AtVertexPropagator::Instance()->GetDecayEvtCnt() && pdgType == 2212 &&
                    fPType.at(i) == "Proton") {
            std::cout << "-I- FairIonGenerator: Generating ions of type " << fParticle.at(i)->GetName() << " (PDG code "
                      << pdgType << ")" << std::endl;
            std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
                      << ") Gev from vertex (" << fVx << ", " << fVy << ", " << fVz << ") cm" << std::endl;
            primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);
         } else if (i > 1 && AtVertexPropagator::Instance()->GetDecayEvtCnt() && pdgType == 2112 &&
                    fPType.at(i) == "Neutron") {
            std::cout << "-I- FairIonGenerator: Generating ions of type " << fParticle.at(i)->GetName() << " (PDG code "
                      << pdgType << ")" << std::endl;
            std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
                      << ") Gev from vertex (" << fVx << ", " << fVy << ", " << fVz << ") cm" << std::endl;
            primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);
         }
      }
   } // if residual energy > 0

   AtVertexPropagator::Instance()
      ->IncDecayEvtCnt(); // TODO: Okay someone should put a more suitable name but we are on a hurry...

   return kTRUE;
}

ClassImp(AtTPCXSReader)
