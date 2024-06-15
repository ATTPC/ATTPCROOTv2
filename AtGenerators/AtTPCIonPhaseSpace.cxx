#include "AtTPCIonPhaseSpace.h"

#include "AtVertexPropagator.h"

#include <FairIon.h>
#include <FairPrimaryGenerator.h>
#include <FairRunSim.h>

#include <TDatabasePDG.h>
#include <TGenPhaseSpace.h>
#include <TLorentzVector.h>
#include <TMath.h>
#include <TParticlePDG.h>
#include <TString.h>
#include <TVector3.h>

#include <algorithm>
#include <cmath>
#include <iostream>

Int_t AtTPCIonPhaseSpace::fgNIon = 0;

AtTPCIonPhaseSpace::AtTPCIonPhaseSpace()
   : fMult(0), fPx(0.), fPy(0.), fPz(0.), fVx(0.), fVy(0.), fVz(0.), fIon(0), fQ(0)
{
   //  cout << "-W- AtTPCIonGenerator: "
   //      << " Please do not use the default constructor! " << endl;
}

// -----   Default constructor   ------------------------------------------
AtTPCIonPhaseSpace::AtTPCIonPhaseSpace(const char *name, std::vector<Int_t> *z, std::vector<Int_t> *a,
                                       std::vector<Int_t> *q, Int_t mult, std::vector<Double_t> *px,
                                       std::vector<Double_t> *py, std::vector<Double_t> *pz,
                                       std::vector<Double_t> *mass, Double_t ResEner, Int_t ZB, Int_t AB, Double_t PxB,
                                       Double_t PyB, Double_t PzB, Double_t BMass, Double_t TMass)
   : fMult(mult), fPx(0.), fPy(0.), fPz(0.), fVx(0.), fVy(0.), fVz(0.), fIon(0), fQ(0), fBeamEnergy_buff(ResEner),
     fBeamMass(BMass), fTargetMass(TMass), fZBeam(ZB), fABeam(AB), fPxBeam(PxB), fPyBeam(PyB), fPzBeam(PzB)
{
   fgNIon++;
   fIon.reserve(fMult);

   for (Int_t i = 0; i < fMult; i++) {

      fPx.push_back(Double_t(a->at(i)) * px->at(i));
      fPy.push_back(Double_t(a->at(i)) * py->at(i));
      fPz.push_back(Double_t(a->at(i)) * pz->at(i));
      Masses.push_back(mass->at(i));

      auto *IonBuff =
         new FairIon(TString::Format("Product_Ion%d", i).Data(), z->at(i), a->at(i), q->at(i), 0.0, mass->at(i));
      // FairIon *IonBuff = new FairIon(buffer, z->at(i), a->at(i), q->at(i));
      // std::cout<<" Z "<<z->at(i)<<" A "<<a->at(i)<<std::endl;
      // std::cout<<buffer<<std::endl;
      std::cout << " Particle " << fMult << " mass " << IonBuff->GetMass() << std::endl;
      fIon.push_back(IonBuff);
   }

   FairRunSim *run = FairRunSim::Instance();
   if (!run) {
      std::cout << "-E- FairIonGenerator: No FairRun instantised!" << std::endl;
      Fatal("FairIonGenerator", "No FairRun instantised!");
   }

   for (Int_t i = 0; i < fMult; i++) {
      run->AddNewIon(fIon.at(i)); // NOLINT
      std::cout << " Z " << z->at(i) << " A " << a->at(i) << std::endl;
      std::cout << fIon.at(i)->GetName() << std::endl;
   }
}

// -----   Public method ReadEvent   --------------------------------------
Bool_t AtTPCIonPhaseSpace::ReadEvent(FairPrimaryGenerator *primGen)
{

   // === Phase Space Calculation
   TLorentzVector fEnergyImpulsionLab_beam;
   TLorentzVector fEnergyImpulsionLab_target;
   TLorentzVector fEnergyImpulsionLab_Total;
   TLorentzVector fEnergyImpulsionFinal;
   TVector3 fImpulsionLab_beam;
   TVector3 fImpulsionLab_target;
   TLorentzVector *p1 = nullptr;
   TLorentzVector *p2 = nullptr;
   TLorentzVector *p3 = nullptr;
   std::vector<TLorentzVector *> p_vector;
   TGenPhaseSpace event1;

   fPx.clear();
   fPy.clear();
   fPx.clear();

   fPx.resize(fMult);
   fPy.resize(fMult);
   fPx.resize(fMult);

   fIsDecay = kFALSE;

   fBeamEnergy = AtVertexPropagator::Instance()->GetEnergy() / 1000.0;
   std::cout << " Residual energy in AtTPCIonPhaseSpace : " << AtVertexPropagator::Instance()->GetEnergy() << std::endl;
   fPxBeam = AtVertexPropagator::Instance()->GetPx();
   fPyBeam = AtVertexPropagator::Instance()->GetPy();
   fPzBeam = AtVertexPropagator::Instance()->GetPz();

   Double_t beta;
   Double_t s = 0.0;
   Double_t mass_1[10] = {0.0};
   Double_t *pMass;

   Double_t M_tot = 0;

   fImpulsionLab_beam = TVector3(fPxBeam, fPyBeam, fPzBeam);
   fEnergyImpulsionLab_beam = TLorentzVector(fImpulsionLab_beam, fBeamMass + fBeamEnergy);
   fEnergyImpulsionLab_target = TLorentzVector(TVector3(0, 0, 0), fTargetMass);

   fEnergyImpulsionLab_Total = fEnergyImpulsionLab_beam + fEnergyImpulsionLab_target;
   s = fEnergyImpulsionLab_Total.M2();

   std::cout << " fABeam : " << fABeam << " fPzBeam : " << fPzBeam << " fBeamEnergy : " << fBeamEnergy << std::endl;

   for (Int_t i = 0; i < fMult; i++) {

      M_tot += Masses.at(i) / 1000.0;
      mass_1[i] = Masses.at(i) / 1000.0;
   }

   std::cout << " S : " << s << " Pow(M) " << pow(M_tot, 2) << std::endl;

   if (s > pow(M_tot, 2)) {

      fIsDecay = kTRUE;

      event1.SetDecay(fEnergyImpulsionLab_Total, fMult, mass_1);
      std::vector<Double_t> KineticEnergy;
      std::vector<Double_t> ThetaLab;

      std::cout << "  ==== Phase Space Information ==== " << std::endl;
      for (Int_t i = 0; i < fMult; i++) {

         p_vector.push_back(event1.GetDecay(i));
         fPx.at(i) = p_vector.at(i)->Px();
         fPy.at(i) = p_vector.at(i)->Py();
         fPz.at(i) = p_vector.at(i)->Pz();
         KineticEnergy.push_back((p_vector.at(i)->E() - mass_1[i]) * 1000);
         ThetaLab.push_back(p_vector.at(i)->Theta() * 180. / TMath::Pi());
         std::cout << " Particle " << i << " - TKE (MeV) : " << KineticEnergy.at(i)
                   << " - Lab Angle (deg) : " << ThetaLab.at(i) << std::endl;
      }

   } // if kinematics condition

   for (Int_t i = 0; i < fMult; i++) {

      TParticlePDG *thisPart = TDatabasePDG::Instance()->GetParticle(fIon.at(i)->GetName());

      if (!thisPart) {
         std::cout << "-W- FairIonGenerator: Ion " << fIon.at(i)->GetName() << " not found in database!" << std::endl;
         return kFALSE;
      }

      int pdgType = thisPart->PdgCode();

      // Propagate the vertex of the previous event

      fVx = AtVertexPropagator::Instance()->GetVx();
      fVy = AtVertexPropagator::Instance()->GetVy();
      fVz = AtVertexPropagator::Instance()->GetVz();

      std::cout << "-I- FairIonGenerator: Generating " << fMult << " with mass " << thisPart->Mass() << " ions of type "
                << fIon.at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl;
      std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i) << ") Gev from vertex ("
                << fVx << ", " << fVy << ", " << fVz << ") cm" << std::endl;

      if (fIsDecay) {
         primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);
      }
   }

   return kTRUE;
}

ClassImp(AtTPCIonPhaseSpace)
