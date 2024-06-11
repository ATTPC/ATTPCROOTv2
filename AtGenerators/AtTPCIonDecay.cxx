/*

*/

#include "AtTPCIonDecay.h"

#include "AtVertexPropagator.h"

#include <FairIon.h>
#include <FairLogger.h>
#include <FairParticle.h>
#include <FairPrimaryGenerator.h>
#include <FairRunSim.h>

#include <TDatabasePDG.h>
#include <TGenPhaseSpace.h>
#include <TLorentzVector.h>
#include <TMath.h>
#include <TParticle.h>
#include <TParticlePDG.h>
#include <TRandom.h>
#include <TString.h>
#include <TVector3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <iterator>
#include <memory>
#include <utility>

constexpr float amu = 931.494;

Int_t AtTPCIonDecay::fgNIon = 0;

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";
constexpr auto cORANGEWARNING = "\033[29;5;202m";
constexpr auto cBLINKINGRED = "\033[32;5m";

AtTPCIonDecay::AtTPCIonDecay()
   : fMult(0), fPx(0.), fPy(0.), fPz(0.), fVx(0.), fVy(0.), fVz(0.), fIon(0), fParticle(0), fPType(0), fNbCases(0),
     fSepEne(0), fMasses(0), fQ(0), fPxBeam(0.), fPyBeam(0.), fPzBeam(0.)
{
   //  cout << "-W- AtTPCIonGenerator: "
   //      << " Please do not use the default constructor! " << endl;
}

// -----   Default constructor   ------------------------------------------
AtTPCIonDecay::AtTPCIonDecay(std::vector<std::vector<Int_t>> *z, std::vector<std::vector<Int_t>> *a,
                             std::vector<std::vector<Int_t>> *q, std::vector<std::vector<Double_t>> *mass, Int_t ZB,
                             Int_t AB, Double_t BMass, Double_t TMass, Double_t ExEnergy, std::vector<Double_t> *SepEne)
   : fMult(0), fNbCases(a->size()), fPx(0.), fPy(0.), fPz(0.), fVx(0.), fVy(0.), fVz(0.), fIon(0), fParticle(0),
     fPType(0), fSepEne(0), fMasses(0), fQ(0), fPxBeam(0.), fPyBeam(0.), fPzBeam(0.)
{

   char buffer[40];

   fgNIon++;

   for (Int_t i = 0; i < fNbCases; i++)
      fMult.push_back(a->at(i).size());
   fParticle.resize(fNbCases);
   fIon.resize(fNbCases);
   fPType.resize(fNbCases);

   fZBeam = ZB;
   fABeam = AB;
   fBeamMass = BMass * amu / 1000.0;
   fTargetMass = TMass * amu / 1000.0;
   fSepEne = SepEne[0];
   fMasses = mass[0];
   fIsSequentialDecay = kFALSE;
   fExEnergy = ExEnergy;

   FairRunSim *run = FairRunSim::Instance();
   if (!run) {
      std::cout << "-E- FairIonGenerator: No FairRun instantised!" << std::endl;
      Fatal("FairIonGenerator", "No FairRun instantised!");
      return;
   }

   for (Int_t k = 0; k < fNbCases; k++) {
      for (Int_t i = 0; i < fMult.at(k); i++) {

         std::unique_ptr<FairIon> IonBuff = nullptr;
         std::unique_ptr<FairParticle> ParticleBuff = nullptr;
         sprintf(buffer, "Product_Ion_dec%d_%d", k, i);
         if (a->at(k).at(i) != 1) {
            IonBuff = std::make_unique<FairIon>(buffer, z->at(k).at(i), a->at(k).at(i), q->at(k).at(i), 0.0,
                                                mass->at(k).at(i) * amu / 1000.0);
            ParticleBuff = std::make_unique<FairParticle>("dummyPart", 1, 1, 1.0, 0, 0.0, 0.0);
            fPType.at(k).push_back("Ion");
            run->AddNewIon(IonBuff.get());

         } else if (a->at(k).at(i) == 1 && z->at(k).at(i) == 1) {
            IonBuff = std::make_unique<FairIon>(buffer, z->at(k).at(i), a->at(k).at(i), q->at(k).at(i), 0.0,
                                                mass->at(k).at(i) * amu / 1000.0);
            auto *kProton = new TParticle(); // NOLINT
            kProton->SetPdgCode(2212);
            ParticleBuff = std::make_unique<FairParticle>(2212, kProton);
            fPType.at(k).push_back("Proton");

         } else if (a->at(k).at(i) == 1 && z->at(k).at(i) == 0) {
            IonBuff = std::make_unique<FairIon>(buffer, z->at(k).at(i), a->at(k).at(i), q->at(k).at(i), 0.0,
                                                mass->at(k).at(i) * amu / 1000.0);
            auto *kNeutron = new TParticle(); // NOLINT
            kNeutron->SetPdgCode(2112);
            ParticleBuff = std::make_unique<FairParticle>(2112, kNeutron);
            fPType.at(k).push_back("Neutron");
         }
         fIon.at(k).push_back(std::move(IonBuff));
         fParticle.at(k).push_back(std::move(ParticleBuff));
      } // for mult
   }    // for case
}

// -----   Public method ReadEvent   --------------------------------------
Bool_t AtTPCIonDecay::ReadEvent(FairPrimaryGenerator *primGen)
{

   Double_t ExEject = AtVertexPropagator::Instance()->GetScatterEx() / 1000.0; // in GeV
   Bool_t IsGoodCase = kFALSE;
   fIsDecay = kFALSE;
   Double_t excitationEnergy = 0.0;

   LOG(info) << cBLUE << " AtTPCIonDecay - Decay energy -  Excitation energy from reaction :  " << ExEject
             << " and from task : " << fExEnergy
             << ". Beam energy : " << AtVertexPropagator::Instance()->GetEnergy() / 1000.0 << " GeV . Is Sequential? "
             << fIsSequentialDecay << cNORMAL << "\n";
   LOG(info) << cORANGEWARNING
             << " AtTPCIonDecay - Warning: Temporary warning message to control the flow of generators.Please, check "
                "that if the decay comes from beam fusion, the energy from reaction is 0"
             << cNORMAL << "\n";

   if (ExEject > 0.0 && !fIsSequentialDecay) {
      LOG(info) << cBLINKINGRED
                << " AtTPCIonDecay - Warning, Inconsistent variables: Recoil excitation energy from Vertex propagator "
                   "greater than 0 but sequential decay not enabled! Continue at your own risk!"
                << cNORMAL << "\n";

   } else if (fIsSequentialDecay && fExEnergy > 0.0) {

      LOG(info) << cBLINKINGRED
                << " AtTPCIonDecay - Warning, Inconsistent variables: Sequential decay should take the Ex energy from "
                   "the reaction generator! Continue at your own risk!"
                << cNORMAL << "\n";

   } else if (ExEject > 0.0 && fExEnergy > 0.0) {
      LOG(info)
         << cBLINKINGRED
         << " AtTPCIonDecay - Warning, Inconsistent variables: Both, excitation energy from Vertex propagator and "
            "excitation energy from task (introduced through the macro) are positive! Continue at your own risk!"
         << cNORMAL << "\n";
   }

   std::vector<Int_t> GoodCases;

   // Test for decay from reaction
   for (Int_t i = 0; i < fNbCases; i++) {
      // if(ExEject*1000.0>fSepEne.at(i)) {
      if (ExEject > -1) { // Forcing all good cases
         GoodCases.push_back(i);
         IsGoodCase = kTRUE;
      }
   }
   if (IsGoodCase) {
      int RandVar = (int)(GoodCases.size()) * gRandom->Uniform();
      auto it = GoodCases.begin();
      std::advance(it, RandVar);
      Int_t Case = *it;
      // LOG(info)<<"iterator "<<" "<<Case<<" "<<RandVar<<" "<<GoodCases.size()<<" "<<std::endl;

      Double_t beta;
      Double_t s = 0.0;
      Double_t mass_1[10] = {0.0};
      Double_t *pMass;
      Double_t M_tot = 0;
      TLorentzVector fEnergyImpulsionLab_beam;
      TLorentzVector fEnergyImpulsionLab_Total;
      TLorentzVector fEnergyImpulsionLab_target;
      TLorentzVector fEnergyImpulsionFinal;

      TVector3 fImpulsionLab_beam;
      std::vector<TLorentzVector *> p_vector;
      TGenPhaseSpace event1;

      fPx.clear();
      fPy.clear();
      fPz.clear();

      fPx.resize(fMult.at(Case));
      fPy.resize(fMult.at(Case));
      fPz.resize(fMult.at(Case));

      // LOG(info)<<" Case : "<<Case<<" with multiplicity : "<<fMult.at(Case)<<"\n";

      fIsDecay = kFALSE;

      if (fIsSequentialDecay) // NB: Decay modelled as two-step (coming from reaction generator)
      {
         fBeamEnergy = AtVertexPropagator::Instance()->GetTrackEnergy(0) / 1000.0;
         TVector3 ScatP = AtVertexPropagator::Instance()->GetScatterP();
         fPxBeam = ScatP.X();
         fPyBeam = ScatP.Y();
         fPzBeam = ScatP.Z();
         excitationEnergy = ExEject; // From ejectile
      } else {                       // simultaneous
         fBeamEnergy = AtVertexPropagator::Instance()->GetEnergy() / 1000.0;
         fPxBeam = AtVertexPropagator::Instance()->GetPx();
         fPyBeam = AtVertexPropagator::Instance()->GetPy();
         fPzBeam = AtVertexPropagator::Instance()->GetPz();
         excitationEnergy = fExEnergy; // From compound nucleus
      }

      TParticlePDG *thisPart0 = nullptr;
      thisPart0 = TDatabasePDG::Instance()->GetParticle(
         fIon.at(Case).at(0)->GetName()); // NB: The first particle of the list must be the decaying ion
      int pdgType0 = thisPart0->PdgCode();

      LOG(info) << cBLUE << " Ejectile info : " << pdgType0 << " " << fBeamMass << " " << fPxBeam << " " << fPyBeam
                << " " << fPzBeam << " " << fBeamEnergy << " " << excitationEnergy << cNORMAL << "\n";

      // === Phase Space Calculation

      fImpulsionLab_beam = TVector3(fPxBeam, fPyBeam, fPzBeam);
      fEnergyImpulsionLab_beam = TLorentzVector(fImpulsionLab_beam, fBeamMass + fBeamEnergy + ExEject);
      fEnergyImpulsionLab_target = TLorentzVector(TVector3(0, 0, 0), fTargetMass);

      if (fTargetMass > 0 && fIsSequentialDecay) {
         LOG(info)
            << cBLINKINGRED
            << " AtTPCIonDecay - Warning, Inconsistent variables: Target Impulsion included in sequential decay. "
               "Continue at your own risk!"
            << cNORMAL << "\n";
      }

      fEnergyImpulsionLab_Total = fEnergyImpulsionLab_beam + fEnergyImpulsionLab_target;

      s = fEnergyImpulsionLab_Total.M2();
      // beta = fEnergyImpulsionLab_Total.Beta();

      for (Int_t i = 0; i < fMult.at(Case); i++) {
         M_tot += fMasses.at(Case).at(i) * amu / 1000.0;
         mass_1[i] = fMasses.at(Case).at(i) * amu / 1000.0;
         std::cout << fMasses.at(Case).at(i) * amu / 1000.0 << " " << M_tot << " " << fBeamMass << " "
                   << fBeamMass - M_tot << " " << ExEject << " " << sqrt(s) << " " << (sqrt(s) - M_tot) * 1000.0
                   << std::endl;
      }

      // std::cout<<" S : "<<s<<" Pow(M) "<<pow(M_tot,2)<<" "<<AtVertexPropagator::Instance()->GetScatterEx()<<"
      // "<<fSepEne.at(Case)<<std::endl;

      if (s > pow(M_tot, 2)) {
         // if(ExEject*1000.0>fSepEne){
         fIsDecay = kTRUE;
         event1.SetDecay(fEnergyImpulsionLab_Total, fMult.at(Case), mass_1);
         event1.Generate(); // to generate the random final state
         std::vector<Double_t> KineticEnergy;
         std::vector<Double_t> ThetaLab;

         LOG(info) << cBLUE << " AtTPCIonDecay -  Phase Space Information "
                   << "\n";
         for (Int_t i = 0; i < fMult.at(Case); i++) {
            p_vector.push_back(event1.GetDecay(i));
            fPx.at(i) = p_vector.at(i)->Px();
            fPy.at(i) = p_vector.at(i)->Py();
            fPz.at(i) = p_vector.at(i)->Pz();
            KineticEnergy.push_back((p_vector.at(i)->E() - mass_1[i]) * 1000.0);
            ThetaLab.push_back(p_vector.at(i)->Theta() * 180. / TMath::Pi());
            LOG(info) << " Particle " << i << " - TKE (MeV) : " << KineticEnergy.at(i)
                      << " - Lab Angle (deg) : " << ThetaLab.at(i) << "\n";
         }
         LOG(info) << cNORMAL;

      } else { // if kinematics condition

         LOG(info) << cYELLOW << "AtTPCIonDecay - Warning, kinematical conditions for decay not fulfilled " << cNORMAL
                   << "\n";
         LOG(info) << cYELLOW << " s = " << s << " - pow(M_tot,2) = " << pow(M_tot, 2) << cNORMAL << "\n";
      }

      // === Propagate the decay products from the vertex of the reaction

      for (Int_t i = 0; i < fMult.at(Case); i++) {
         TParticlePDG *thisPart = nullptr;

         if (fPType.at(Case).at(i) == "Ion")
            thisPart = TDatabasePDG::Instance()->GetParticle(fIon.at(Case).at(i)->GetName());
         else if (fPType.at(Case).at(i) == "Proton")
            thisPart = TDatabasePDG::Instance()->GetParticle(fParticle.at(Case).at(i)->GetName());
         else if (fPType.at(Case).at(i) == "Neutron")
            thisPart = TDatabasePDG::Instance()->GetParticle(fParticle.at(Case).at(i)->GetName());

         if (!thisPart) {
            if (fPType.at(Case).at(i) == "Ion")
               std::cout << "-W- FairIonGenerator: Ion " << fIon.at(Case).at(i)->GetName() << " not found in database!"
                         << std::endl;
            else if (fPType.at(Case).at(i) == "Proton")
               std::cout << "-W- FairIonGenerator: Particle " << fParticle.at(Case).at(i)->GetName()
                         << " not found in database!" << std::endl;
            else if (fPType.at(Case).at(i) == "Neutron")
               std::cout << "-W- FairIonGenerator: Particle " << fParticle.at(Case).at(i)->GetName()
                         << " not found in database!" << std::endl;
            return kFALSE;
         }

         int pdgType = thisPart->PdgCode();

         if (AtVertexPropagator::Instance()->Getd2HeEvt()) {
            TVector3 d2HeVtx = AtVertexPropagator::Instance()->Getd2HeVtx();
            fVx = d2HeVtx.X();
            fVy = d2HeVtx.Y();
            fVz = d2HeVtx.Z();
         } else {
            fVx = AtVertexPropagator::Instance()->GetVx();
            fVy = AtVertexPropagator::Instance()->GetVy();
            fVz = AtVertexPropagator::Instance()->GetVz();
         }

         // std::cout << "-I- FairIonGenerator: Generating " <<" with mass "<<thisPart->Mass()<<" ions of type "<<
         // fIon.at(Case).at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl; std::cout << "    Momentum
         // (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
         // << ") Gev from vertex (" << fVx << ", " << fVy
         // << ", " << fVz << ") cm" << std::endl;

         if (fIsDecay) {
            primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);
         }

      } // for fMult.at(Case)
   }    // if IsGoodCase

   // if (!fIsSequentialDecay)
   AtVertexPropagator::Instance()->IncDecayEvtCnt(); // Increase count only if no other generator is meant to do it.

   return kTRUE;
}

ClassImp(AtTPCIonDecay)
