#include "AtTPC_d2He.h"

#include "AtVertexPropagator.h"

#include <FairIon.h>
#include <FairParticle.h>
#include <FairPrimaryGenerator.h>
#include <FairRunSim.h>

#include <TDatabasePDG.h>
#include <TMath.h>
#include <TMathBase.h>
#include <TParticle.h>
#include <TParticlePDG.h>
#include <TRandom.h>
#include <TVector3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>

constexpr auto cRED = "\033[1;31m";
constexpr auto cYELLOW = "\033[1;33m";
constexpr auto cNORMAL = "\033[0m";
constexpr auto cGREEN = "\033[1;32m";
constexpr auto cBLUE = "\033[1;34m";

constexpr float amu = 931.494;

Int_t AtTPC_d2He::fgNIon = 0;

AtTPC_d2He::AtTPC_d2He() : fMult(0), fPx(0.), fPy(0.), fPz(0.), fVx(0.), fVy(0.), fVz(0.), fIon(0)
{
   //  cout << "-W- AtTPCIonGenerator: "
   //      << " Please do not use the default constructor! " << endl;
}

inline Double_t sign(Double_t num)
{
   if (num > 0)
      return 1;
   return (num == 0) ? 1 : -1;
}

// -----   Default constructor   ------------------------------------------
AtTPC_d2He::AtTPC_d2He(const char *name, std::vector<Int_t> *z, std::vector<Int_t> *a, std::vector<Int_t> *q,
                       Int_t mult, std::vector<Double_t> *px, std::vector<Double_t> *py, std::vector<Double_t> *pz,
                       std::vector<Double_t> *mass, std::vector<Double_t> *Ex, std::vector<Double_t> *cross1,
                       std::vector<Double_t> *cross2, std::vector<Double_t> *cross3, Int_t N_data)
   : fPx(0.), fPy(0.), fPz(0.), fMult(mult), fVx(0.), fVy(0.), fVz(0.), fIon(0)
{

   fgNIon++;

   fIon.reserve(fMult);

   char buffer[30];
   auto *kProton = new TParticle(); // NOLINT but probably a problem
   kProton->SetPdgCode(2212);

   auto *kNeutron = new TParticle(); // NOLINT but probably a problem
   kNeutron->SetPdgCode(2112);

   // Read the cross section table
   for (Int_t i = 0; i < N_data; i++) {
      inp1.push_back(cross1->at(i));
      inp2.push_back(cross2->at(i));
      inp3.push_back(cross3->at(i));
      fCStot += inp3.at(i);
      // std::cout<<"===================================================================="<<std::endl;
      // std::cout<<inp1.at(i)<<"  "<<inp2.at(i)<<"  "<<inp3.at(i)<<std::endl;
      // std::cout<<"===================================================================="<<std::endl;
   }

   fN = N_data;

   for (Int_t i = 0; i < fMult; i++) {

      fPx.push_back(px->at(i));
      fPy.push_back(py->at(i));
      fPz.push_back(pz->at(i));
      Masses.push_back(mass->at(i));
      fExEnergy.push_back(Ex->at(i));
      // fWm.push_back( mass->at(i));

      FairIon *IonBuff;
      FairParticle *ParticleBuff;
      sprintf(buffer, "Product_Ion%d", i);
      if (a->at(i) != 1) {
         IonBuff = new FairIon(buffer, z->at(i), a->at(i), q->at(i), 0.0, // NOLINT but probably a problem
                               mass->at(i) * amu / 1000.0);
         ParticleBuff = new FairParticle("dummyPart", 1, 1, 1.0, 0, 0.0, 0.0); // NOLINT but probably a problem
         fPType.emplace_back("Ion");
         //          std::cout<<" Adding : "<<buffer<<std::endl;

      } else if (a->at(i) == 1 && z->at(i) == 1) {

         IonBuff = new FairIon(buffer, z->at(i), a->at(i), q->at(i), 0.0, // NOLINT but probably a problem
                               mass->at(i) * amu / 1000.0);
         ParticleBuff = new FairParticle(2212, kProton); // NOLINT but probably a problem
         fPType.emplace_back("Proton");

      } else if (a->at(i) == 1 && z->at(i) == 0) {

         IonBuff = new FairIon(buffer, z->at(i), a->at(i), q->at(i), 0.0, // NOLINT but probably a problem
                               mass->at(i) * amu / 1000.0);
         ParticleBuff = new FairParticle(2112, kNeutron); // NOLINT but probably a problem
         fPType.emplace_back("Neutron");
      }

      //	       std::cout<<" Z "<<z->at(i)<<" A "<<a->at(i)<<std::endl;
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
         //                 std::cout<<" In position "<<i<<" adding an : "<<fPType.at(i)<<std::endl;
         run->AddNewIon(fIon.at(i)); // NOLINT
         //		             std::cout<<" fIon name :"<<fIon.at(i)->GetName()<<std::endl;
         //                 std::cout<<" fParticle name :"<<fParticle.at(i)->GetName()<<std::endl;

      } else if (fPType.at(i) == "Proton") {
         //		             std::cout<<" In position "<<i<<" adding an : "<<fPType.at(i)<<std::endl;
         // run->AddNewParticle(fParticle.at(i));
         //                 std::cout<<" fIon name :"<<fIon.at(i)->GetName()<<std::endl;
         //                 std::cout<<" fParticle name :"<<fParticle.at(i)->GetName()<<std::endl;
         //                 std::cout<<fParticle.at(i)->GetName()<<std::endl;

      } else if (fPType.at(i) == "Neutron") {

         //                 std::cout<<" In position "<<i<<" adding an : "<<fPType.at(i)<<std::endl;
         // run->AddNewParticle(fParticle.at(i));
         //                 std::cout<<" fIon name :"<<fIon.at(i)->GetName()<<std::endl;
         //                 std::cout<<" fParticle name :"<<fParticle.at(i)->GetName()<<std::endl;
         //                 std::cout<<fParticle.at(i)->GetName()<<std::endl;
      }
   }
}

// Rotation of a 3D vector around an arbitrary axis
// Rodriges Formula
std::vector<Double_t>
AtTPC_d2He::TRANSF(std::vector<Double_t> *from, std::vector<Double_t> *to, std::vector<Double_t> *vin)
{

   static std::vector<double> vout(3);
   double n[3];
   double normn, normf, normt;
   double alpha, a, b;

   normf = sqrt(pow(from->at(0), 2) + pow(from->at(1), 2) + pow(from->at(2), 2));
   normt = sqrt(pow(to->at(0), 2) + pow(to->at(1), 2) + pow(to->at(2), 2));

   alpha =
      acos(((from->at(0)) * (to->at(0)) + (from->at(1)) * (to->at(1)) + (from->at(2)) * (to->at(2))) / (normf * normt));
   a = sin(alpha);
   b = 1.0 - cos(alpha);

   if (fabs(alpha) < 0.000001) {
      vout.at(0) = vin->at(0);
      vout.at(1) = vin->at(1);
      vout.at(2) = vin->at(2);

   } else {
      n[0] = ((from->at(1)) * (to->at(2)) - (from->at(2)) * (to->at(1)));
      n[1] = ((from->at(2)) * (to->at(0)) - (from->at(0)) * (to->at(2)));
      n[2] = ((from->at(0)) * (to->at(1)) - (from->at(1)) * (to->at(0)));

      // std::cout<<from->at(0)<<" "<<from->at(1)<<" "<<from->at(2)<<std::endl;
      // std::cout<<to->at(0)<<" "<<to->at(1)<<" "<<to->at(2)<<std::endl;
      // std::cout<<vin->at(0)<<" "<<vin->at(1)<<" "<<vin->at(2)<<std::endl;
      normn = sqrt(pow(n[0], 2) + pow(n[1], 2) + pow(n[2], 2));
      n[0] = n[0] / normn;
      n[1] = n[1] / normn;
      n[2] = n[2] / normn;

      vout.at(0) = (1 - b * (pow(n[2], 2) + pow(n[1], 2))) * (vin->at(0)) +
                   (-a * n[2] + b * n[0] * n[1]) * (vin->at(1)) + (a * n[1] + b * n[0] * n[2]) * (vin->at(2));
      vout.at(1) = (a * n[2] + b * n[0] * n[1]) * (vin->at(0)) +
                   (1 - b * (pow(n[2], 2) + pow(n[0], 2))) * (vin->at(1)) +
                   (-a * n[0] + b * n[1] * n[2]) * (vin->at(2));
      vout.at(2) = (-a * n[1] + b * n[0] * n[2]) * (vin->at(0)) + (a * n[0] + b * n[1] * n[2]) * (vin->at(1)) +
                   (1 - b * (pow(n[1], 2) + pow(n[0], 2))) * (vin->at(2));
   }

   return vout;
}

Double_t AtTPC_d2He::omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

// -----   Public method ReadEvent   --------------------------------------
Bool_t AtTPC_d2He::ReadEvent(FairPrimaryGenerator *primGen)
{

   std::vector<Double_t> Ang; // Lab Angle of the products
   std::vector<Double_t> Ene; // Lab Energy of the products
   Ang.resize(4);
   Ene.resize(4);

   fIsDecay = kFALSE;

   fBeamEnergy = AtVertexPropagator::Instance()->GetEnergy();
   std::cout << " -I- AtTPC_d2He Residual energy  : " << AtVertexPropagator::Instance()->GetEnergy() << std::endl;

   fPxBeam = AtVertexPropagator::Instance()->GetPx();
   fPyBeam = AtVertexPropagator::Instance()->GetPy();
   fPzBeam = AtVertexPropagator::Instance()->GetPz();

   // fPxBeam = fPx.at(0) ;
   // fPyBeam = fPy.at(0) ;
   // fPzBeam = fPz.at(0) ;

   if (fBeamEnergy == 0) {
      std::cout << "-I- AtTP_d2He : No solution!" << std::endl;
      AtVertexPropagator::Instance()->SetValidKine(kFALSE);
   }

   if (!AtVertexPropagator::Instance()->GetValidKine()) {

      fPx.at(2) = 0.; // To GeV for FairRoot
      fPy.at(2) = 0.;
      fPz.at(2) = 0.;

      fPx.at(3) = 0.;
      fPy.at(3) = 0.;
      fPz.at(3) = 0.;

      fPx.at(4) = 0.;
      fPy.at(4) = 0.;
      fPz.at(4) = 0.;

      fPx.at(5) = 0.;
      fPy.at(5) = 0.;
      fPz.at(5) = 0.;

      Ene.at(0) = 0.0;
      Ang.at(0) = 0.0;
      Ene.at(1) = 0.0;
      Ang.at(1) = 0.0;
      Ene.at(2) = 0.0;
      Ang.at(2) = 0.0;
      Ene.at(3) = 0.0;
      Ang.at(3) = 0.0;
   }

   else {
      // MC to distribute the events with the cross section
      // fN==1 used for the efficiency map (1 simu per theta-epp bin)
      if (fN == 1) { // Depp=0.25 and Dtheta_cm=0.5 in ACCBA file
         if (inp2.at(0) == 0)
            theta_cm = (inp2.at(0) + 0.25 * gRandom->Uniform()) *
                       TMath::DegToRad(); // carefull in the analysis, these bins have 2 times more stat
         else
            theta_cm = (inp2.at(0) - 0.25 + 0.5 * gRandom->Uniform()) * TMath::DegToRad();
         phi_cm = 2 * TMath::Pi() * (gRandom->Uniform());
         epsilon = inp1.at(0) - 0.125 + 0.25 * gRandom->Uniform();
      } else {
         do {
            ran_theta = fN * gRandom->Uniform();
            ranX = fCStot * gRandom->Uniform();
         } while (ranX > inp3.at(ran_theta));

         // Depp=0.25 and Dtheta_cm=0.5 in ACCBA file, here Depp=0.5 and Dtheta_cm=1 introduce smearing
         theta_cm = TMath::Abs(inp2.at(ran_theta) - 0.5 + gRandom->Uniform()) * TMath::DegToRad();
         phi_cm = 2 * TMath::Pi() * (gRandom->Uniform());
         epsilon = TMath::Abs(inp1.at(ran_theta) - 0.25 + 0.5 * gRandom->Uniform());
      }
      // std::cout<<"===================================================================="<<std::endl;
      // std::cout<<theta_cm*TMath::RadToDeg()<<"  "<<phi_cm<<"  "<<epsilon<<std::endl;
      // std::cout<<fCStot<<std::endl;
      // //std::cout<<fPxBeam<<"  "<<fPyBeam<<"  "<<fPzBeam<<std::endl;
      // std::cout<<"===================================================================="<<std::endl;

      // dirty way to include more than one excited state
      /*test_var = gRandom->Uniform();
      if(test_var>= 0 && test_var<0.25) Ex_ejectile = 0.0;
      if(test_var>= 0.25 && test_var<0.50) Ex_ejectile = 5.0;
      if(test_var>= 0.50 && test_var<0.75) Ex_ejectile = 10.0;
      if(test_var>= 0.75 && test_var<1.0) Ex_ejectile = 20.0;
      */
      Ex_ejectile = fExEnergy.at(2);

      m1 = Masses.at(0) * amu + fExEnergy.at(0);
      m2 = Masses.at(1) * amu + fExEnergy.at(1);
      m3 = Masses.at(2) * amu + Ex_ejectile; // ejectile
      m4 = Masses.at(3) * amu + epsilon;     // 2he
      m7 = Masses.at(4) * amu;
      m8 = Masses.at(5) * amu;
      // K1 = sqrt(pow(fPx.at(0),2) + pow( fPy.at(0),2) + pow( fPz.at(0),2) + pow(m1,2)) - m1;
      K1 = sqrt(pow(fPxBeam * 1000.0, 2) + pow(fPyBeam * 1000.0, 2) + pow(fPzBeam * 1000.0, 2) + pow(m1, 2)) - m1;

      p1L[0] = fPxBeam * 1000.0;
      p1L[1] = fPyBeam * 1000.0;
      p1L[2] = fPzBeam * 1000.0;
      // p1L[0] = fPx.at(0);
      // p1L[1] = fPy.at(0);
      // p1L[2] = fPz.at(0);

      E1L = K1 + m1;

      //  get COM parameters
      beta_cm = p1L[2] / (E1L + m2);
      gamma_cm = 1.0 / sqrt(1.0 - pow(beta_cm, 2));
      S = 2. * E1L * m2 + pow(m1, 2) + pow(m2, 2);
      Pcm = 0.5 * (AtTPC_d2He::omega(S, pow(m3, 2), pow(m4, 2))) / sqrt(S);

      // std::cout<<beta_cm<<"  "<<gamma_cm<<"  "<<p1L[2]<<"  "<<K1<<"  "<<m1<<" "<<(AtTPC_d2He::omega(S, pow(m3,2),
      // pow(m4,2)))<<" "<<sqrt(S)<<std::endl;

      // generate cm angles and momenta (for now isotropic) and corresponding momenta
      p4C[2] = Pcm * cos(TMath::Pi() - theta_cm); // Pi -thethacm because inv kinematics
      p4C[0] = Pcm * sin(TMath::Pi() - theta_cm) * cos(phi_cm);
      p4C[1] = Pcm * sin(TMath::Pi() - theta_cm) * sin(phi_cm);
      E4C = sqrt(pow(Pcm, 2) + pow(m4, 2));

      for (int i = 0; i < 3; i++) {
         p3C[i] = -1. * p4C[i];
      }
      E3C = sqrt(pow(Pcm, 2) + pow(m3, 2));

      // transformation to the lab frame
      p3L[0] = p3C[0];
      p3L[1] = p3C[1];
      p3L[2] = gamma_cm * (p3C[2] + beta_cm * E3C);
      E3L = sqrt(pow(p3L[0], 2) + pow(p3L[1], 2) + pow(p3L[2], 2) + pow(m3, 2));

      p4L[0] = p4C[0];
      p4L[1] = p4C[1];
      p4L[2] = gamma_cm * (p4C[2] + beta_cm * E4C);
      E4L = sqrt(pow(p4L[0], 2) + pow(p4L[1], 2) + pow(p4L[2], 2) + pow(m4, 2));

      // std::cout<<E3L-m3<<" "<<E4L-m4<<" "<<p3L[2]<<" "<<p4L[2]<<std::endl;

      // rotate back to the beam axis
      fvto.clear();
      fvto.resize(3);
      fvin.clear();
      fvin.resize(3);
      fvout.clear();
      fvout.resize(3);
      fvfrom.clear();
      fvfrom.resize(3);
      fvfrom.at(0) = 0;
      fvfrom.at(1) = 0;
      fvfrom.at(2) = 1;
      fvto.at(0) = p1L[0];
      fvto.at(1) = p1L[1];
      fvto.at(2) = p1L[2];
      fvin.at(0) = p3L[0];
      fvin.at(1) = p3L[1];
      fvin.at(2) = p3L[2];
      fvout = AtTPC_d2He::TRANSF(&fvfrom, &fvto, &fvin);
      p3L[0] = fvout.at(0);
      p3L[1] = fvout.at(1);
      p3L[2] = fvout.at(2);
      E3L = sqrt(pow(p3L[0], 2) + pow(p3L[1], 2) + pow(p3L[2], 2) + pow(m3, 2));

      fvin.at(0) = p4L[0];
      fvin.at(1) = p4L[1];
      fvin.at(2) = p4L[2];
      fvout = AtTPC_d2He::TRANSF(&fvfrom, &fvto, &fvin);
      p4L[0] = fvout.at(0);
      p4L[1] = fvout.at(1);
      p4L[2] = fvout.at(2);
      E4L = sqrt(pow(p4L[0], 2) + pow(p4L[1], 2) + pow(p4L[2], 2) + pow(m4, 2));

      // Particle 4 is the 2He which is unbound and will thus decay into 2 protons
      normP4L = sqrt(pow(p4L[0], 2) + pow(p4L[1], 2) + pow(p4L[2], 2));
      beta4 = normP4L / E4L;
      gamma4 = 1.0 / sqrt(1.0 - pow(beta4, 2));
      S_78 = pow(m4, 2);
      Pc78 = 0.5 * AtTPC_d2He::omega(S_78, pow(m7, 2), pow(m8, 2)) / sqrt(S_78);

      //-----------generate isotropically theta and phi of particles 7 and 8
      ran1 = (gRandom->Uniform());
      ran2 = (gRandom->Uniform());
      theta78 = acos(2 * ran1 - 1.);
      phi78 = 2 * TMath::Pi() * ran2;

      // generate the protons in the 2He rest frame
      p7rest[2] = Pc78 * cos(theta78);
      p7rest[0] = Pc78 * sin(theta78) * cos(phi78);
      p7rest[1] = Pc78 * sin(theta78) * sin(phi78);
      p8rest[2] = -1 * p7rest[2];
      p8rest[0] = -1 * p7rest[0];
      p8rest[1] = -1 * p7rest[1];
      E7rest = sqrt(pow(Pc78, 2) + pow(m7, 2));
      E8rest = sqrt(pow(Pc78, 2) + pow(m8, 2));

      // boost to 2He frame
      p7L[0] = p7rest[0];
      p7L[1] = p7rest[1];
      p7L[2] = gamma4 * (p7rest[2] + beta4 * E7rest);
      E7L = sqrt(pow(m7, 2) + pow(p7L[0], 2) + pow(p7L[1], 2) + pow(p7L[2], 2));

      p8L[0] = p8rest[0];
      p8L[1] = p8rest[1];
      p8L[2] = gamma4 * (p8rest[2] + beta4 * E8rest);
      E8L = sqrt(pow(m8, 2) + pow(p8L[0], 2) + pow(p8L[1], 2) + pow(p8L[2], 2));

      // rotate to the 2He direction
      fvto.at(0) = p4L[0];
      fvto.at(1) = p4L[1];
      fvto.at(2) = p4L[2];
      fvin.at(0) = p7L[0];
      fvin.at(1) = p7L[1];
      fvin.at(2) = p7L[2];
      fvout = AtTPC_d2He::TRANSF(&fvfrom, &fvto, &fvin);
      p7L[0] = fvout.at(0);
      p7L[1] = fvout.at(1);
      p7L[2] = fvout.at(2);

      fvin.at(0) = p8L[0];
      fvin.at(1) = p8L[1];
      fvin.at(2) = p8L[2];
      fvout = AtTPC_d2He::TRANSF(&fvfrom, &fvto, &fvin);
      p8L[0] = fvout.at(0);
      p8L[1] = fvout.at(1);
      p8L[2] = fvout.at(2);

      E7L = sqrt(pow(m7, 2) + pow(p7L[0], 2) + pow(p7L[1], 2) + pow(p7L[2], 2));
      E8L = sqrt(pow(m8, 2) + pow(p8L[0], 2) + pow(p8L[1], 2) + pow(p8L[2], 2));

      fPx.at(2) = p3L[0] / 1000.0; // To GeV for FairRoot
      fPy.at(2) = p3L[1] / 1000.0;
      fPz.at(2) = p3L[2] / 1000.0;

      fPx.at(3) = p4L[0] / 1000.0; // To GeV for FairRoot
      fPy.at(3) = p4L[1] / 1000.0;
      fPz.at(3) = p4L[2] / 1000.0;

      fPx.at(4) = p7L[0] / 1000.0; // To GeV for FairRoot
      fPy.at(4) = p7L[1] / 1000.0;
      fPz.at(4) = p7L[2] / 1000.0;

      fPx.at(5) = p8L[0] / 1000.0; // To GeV for FairRoot
      fPy.at(5) = p8L[1] / 1000.0;
      fPz.at(5) = p8L[2] / 1000.0;

      Ene.at(0) = E3L - m3; // beam like particle
      Ang.at(0) = acos((p1L[0] * p3L[0] + p1L[1] * p3L[1] + p1L[2] * p3L[2]) /
                       (sqrt(pow(p1L[0], 2) + pow(p1L[1], 2) + pow(p1L[2], 2)) *
                        sqrt(pow(p3L[0], 2) + pow(p3L[1], 2) + pow(p3L[2], 2)))) *
                  TMath::RadToDeg();
      Ene.at(1) = E4L - m4; // 2He
      Ang.at(1) = acos((p1L[0] * p4L[0] + p1L[1] * p4L[1] + p1L[2] * p4L[2]) /
                       (sqrt(pow(p1L[0], 2) + pow(p1L[1], 2) + pow(p1L[2], 2)) *
                        sqrt(pow(p4L[0], 2) + pow(p4L[1], 2) + pow(p4L[2], 2)))) *
                  TMath::RadToDeg();
      Ene.at(2) = E7L - m7; // proton 1
      Ang.at(2) = acos((p1L[0] * p7L[0] + p1L[1] * p7L[1] + p1L[2] * p7L[2]) /
                       (sqrt(pow(p1L[0], 2) + pow(p1L[1], 2) + pow(p1L[2], 2)) *
                        sqrt(pow(p7L[0], 2) + pow(p7L[1], 2) + pow(p7L[2], 2)))) *
                  TMath::RadToDeg();
      Ene.at(3) = E8L - m8;
      Ang.at(3) = acos((p1L[0] * p8L[0] + p1L[1] * p8L[1] + p1L[2] * p8L[2]) /
                       (sqrt(pow(p1L[0], 2) + pow(p1L[1], 2) + pow(p1L[2], 2)) *
                        sqrt(pow(p8L[0], 2) + pow(p8L[1], 2) + pow(p8L[2], 2)))) *
                  TMath::RadToDeg();

      if (std::isnan(Ene.at(0)) || std::isnan(Ene.at(1)) || std::isnan(Ene.at(2)) || std::isnan(Ene.at(3))) {

         fPx.at(2) = 0.; // To GeV for FairRoot
         fPy.at(2) = 0.;
         fPz.at(2) = 0.;

         fPx.at(3) = 0.;
         fPy.at(3) = 0.;
         fPz.at(3) = 0.;

         fPx.at(4) = 0.;
         fPy.at(4) = 0.;
         fPz.at(4) = 0.;

         fPx.at(5) = 0.;
         fPy.at(5) = 0.;
         fPz.at(5) = 0.;
      }

   } // if solution is valid

   /*
      Double_t phi7 = atan2(p7L[1], p7L[0]) * TMath::RadToDeg();
      if (phi7 < 0)
         phi7 = (phi7 + 360.0);

      Double_t phi8 = atan2(p8L[1], p8L[0]) * TMath::RadToDeg();
      if (phi8 < 0)
         phi8 = (phi8 + 360.0);
   */
   // std::cout << " -I- ===== AtTPC_d2He - Kinematics ====== "<<Ex_ejectile<<std::endl;
   // std::cout << " Scattered energy:" << Ene.at(0)  << " MeV" << std::endl;
   // std::cout << " Scattered  angle:"  << Ang.at(0) << " deg" << std::endl;
   // std::cout << " proton1 energy:" << Ene.at(2) << " MeV" << std::endl;
   // std::cout << " proton1 angle:"  << Ang.at(2) << " deg" << std::endl;
   // std::cout << " proton1 angle phi:"  << phi78 << " deg" << std::endl;
   // std::cout << " proton2 energy:" << Ene.at(3) << " MeV" << std::endl;
   // std::cout << " proton2 angle:"  << Ang.at(3) << " deg" << std::endl;
   // // std::cout << " proton2 angle phi:"  << phi78 << " deg" << std::endl;
   // std::cout << " 2He kinetic energy:"  <<  Ene.at(1) << " MeV" << std::endl;
   // std::cout << " 2He lab angle:"  <<  Ang.at(1) << " deg" << std::endl;

   AtVertexPropagator::Instance()->SetTrackEnergy(0, Ene.at(0));
   AtVertexPropagator::Instance()->SetTrackAngle(0, Ang.at(0));
   AtVertexPropagator::Instance()->SetTrackEnergy(1, Ene.at(2));
   AtVertexPropagator::Instance()->SetTrackAngle(1, Ang.at(2));
   AtVertexPropagator::Instance()->SetTrackEnergy(2, Ene.at(3));
   AtVertexPropagator::Instance()->SetTrackAngle(2, Ang.at(3));

   TVector3 ScatP(fPx.at(2), fPy.at(2), fPz.at(2));
   AtVertexPropagator::Instance()->SetScatterP(ScatP);
   AtVertexPropagator::Instance()->SetScatterEx(Ex_ejectile);

   /*
       do{
         random_z = 100.0*(gRandom->Uniform()); //cm

         random_r = 1.0*(gRandom->Gaus(0,1)); //cm
         random_phi = 2.0*TMath::Pi()*(gRandom->Uniform()); //rad

       }while(  fabs(random_r) > 4.7 ); //cut at 2 sigma
   */
   TVector3 d2HeVtx = AtVertexPropagator::Instance()->Getd2HeVtx();
   fVx = d2HeVtx.X();
   fVy = d2HeVtx.Y();
   fVz = d2HeVtx.Z();
   std::cout << cYELLOW << "vertex in AtTPC_d2He " << fVx << " " << fVy << " " << fVz << cNORMAL << std::endl;

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

      // fVx = AtVertexPropagator::Instance()->GetVx();
      // fVy = AtVertexPropagator::Instance()->GetVy();
      // fVz = AtVertexPropagator::Instance()->GetVz();

      /*
             fVx = random_r*cos(random_phi);
             fVy = random_r*sin(random_phi);
             fVz =  random_z;
      */
      // cout<<AtVertexPropagator::Instance()->GetVx(); <<" "<<AtVertexPropagator::Instance()->GetVy();<<"
      // "<<AtVertexPropagator::Instance()->GetVz();<<" "<<endl;

      // TVector3 d2HeVtx(fVx,fVy,fVz);
      // AtVertexPropagator::Instance()->Setd2HeVtx(d2HeVtx);

      if (i > 1 && i != 3 && AtVertexPropagator::Instance()->GetDecayEvtCnt() && pdgType != 1000500500 &&
          fPType.at(i) == "Ion") {
         // TODO: Dirty way to propagate only the products (0 and 1 are beam and target respectively)
         // i=3 is excluded because  corresponds to 2He
         /*			            std::cout << "-I- FairIonGenerator: Generating ions of type "
         << fIon.at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl;
         std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
         << ") Gev from vertex (" << fVx << ", " << fVy
         << ", " << fVz << ") cm" << std::endl;*/

         primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);

      } else if (i > 1 && i != 3 && AtVertexPropagator::Instance()->GetDecayEvtCnt() && pdgType == 2212 &&
                 fPType.at(i) == "Proton") {

         /*  			      std::cout << "-I- FairIonGenerator: Generating ions of type "
           << fParticle.at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl;
           std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
           << ") Gev from vertex (" << fVx << ", " << fVy
           << ", " << fVz << ") cm" << std::endl;*/

         // primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);
         primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);

      } else if (i > 1 && i != 3 && AtVertexPropagator::Instance()->GetDecayEvtCnt() && pdgType == 2112 &&
                 fPType.at(i) == "Neutron") {

         /*         std::cout << "-I- FairIonGenerator: Generating ions of type "
         << fParticle.at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl;
         std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i)
         << ") Gev from vertex (" << fVx << ", " << fVy
         << ", " << fVz << ") cm" << std::endl;
         */
         primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);
      }
   }

   AtVertexPropagator::Instance()
      ->IncDecayEvtCnt(); // TODO: Okay someone should put a more suitable name but we are on a hurry...
   AtVertexPropagator::Instance()->Getd2HeEvt();

   return kTRUE;
}

ClassImp(AtTPC_d2He)
