#include "AtTPC_Background.h"

#include "AtVertexPropagator.h"

#include <FairIon.h>
#include <FairParticle.h>
#include <FairPrimaryGenerator.h>
#include <FairRunSim.h>

#include <TMath.h>
#include <TParticle.h>
#include <TRandom.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <utility> // for move

constexpr float amu = 931.494;

Int_t AtTPC_Background::fgNIon = 0;

AtTPC_Background::AtTPC_Background() : fMult(0), fPx(0.), fPy(0.), fPz(0.), fVx(0.), fVy(0.), fVz(0.), fIon(0)
{
   //  cout << "-W- AtTPCIonGenerator: "
   //      << " Please do not use the default constructor! " << endl;
}

// -----   Default constructor   ------------------------------------------
AtTPC_Background::AtTPC_Background(const char *name, std::vector<Int_t> *z, std::vector<Int_t> *a,
                                   std::vector<Int_t> *q, Int_t mult, std::vector<Double_t> *px,
                                   std::vector<Double_t> *py, std::vector<Double_t> *pz, std::vector<Double_t> *mass,
                                   std::vector<Double_t> *Ex)
   : fPx(0.), fPy(0.), fPz(0.), fMult(mult), fVx(0.), fVy(0.), fVz(0.), fIon(0)
{

   fgNIon++;

   fIon.reserve(fMult);

   char buffer[30];

   for (Int_t i = 0; i < fMult; i++) {

      fPx.push_back(px->at(i));
      fPy.push_back(py->at(i));
      fPz.push_back(pz->at(i));
      Masses.push_back(mass->at(i));
      fExEnergy.push_back(Ex->at(i));
      // fWm.push_back( mass->at(i));

      std::unique_ptr<FairIon> IonBuff = nullptr;
      std::unique_ptr<FairParticle> ParticleBuff = nullptr;
      sprintf(buffer, "Product_Ion%d", i);
      if (a->at(i) != 1) {
         IonBuff = std::make_unique<FairIon>(buffer, z->at(i), a->at(i), q->at(i), 0.0, mass->at(i) * amu / 1000.0);
         ParticleBuff = std::make_unique<FairParticle>("dummyPart", 1, 1, 1.0, 0, 0.0, 0.0);
         fPType.emplace_back("Ion");
         std::cout << " Adding : " << buffer << std::endl;

      } else if (a->at(i) == 1 && z->at(i) == 1) {

         IonBuff = std::make_unique<FairIon>(buffer, z->at(i), a->at(i), q->at(i), 0.0, mass->at(i) * amu / 1000.0);
         auto *kProton = new TParticle(); // NOLINT
         kProton->SetPdgCode(2212);
         ParticleBuff = std::make_unique<FairParticle>(2212, kProton);
         fPType.emplace_back("Proton");
      }

      std::cout << " Z " << z->at(i) << " A " << a->at(i) << std::endl;
      // std::cout<<buffer<<std::endl;
      fIon.push_back(std::move(IonBuff));
      fParticle.push_back(std::move(ParticleBuff));
   }

   FairRunSim *run = FairRunSim::Instance();
   if (!run) {
      std::cout << "-E- FairIonGenerator: No FairRun instantised!" << std::endl;
      Fatal("FairIonGenerator", "No FairRun instantised!");
   }

   for (Int_t i = 0; i < fMult; i++) {

      if (fPType.at(i) == "Ion") {
         std::cout << " In position " << i << " adding an : " << fPType.at(i) << std::endl;
         run->AddNewIon(fIon.at(i).get()); // NOLINT
         std::cout << " fIon name :" << fIon.at(i)->GetName() << std::endl;
         std::cout << " fParticle name :" << fParticle.at(i)->GetName() << std::endl;

      } else if (fPType.at(i) == "Proton") {
         std::cout << " In position " << i << " adding an : " << fPType.at(i) << std::endl;
         run->AddNewParticle(fParticle.at(i).get()); // NOLINT
         std::cout << " fIon name :" << fIon.at(i)->GetName() << std::endl;
         std::cout << " fParticle name :" << fParticle.at(i)->GetName() << std::endl;
         std::cout << fParticle.at(i)->GetName() << std::endl;
      }
   }
}

Double_t AtTPC_Background::omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

// Rotation of a 3D vector around an arbitrary axis
// Rodriges Formula
std::vector<Double_t>
AtTPC_Background::TRANSF(std::vector<Double_t> *from, std::vector<Double_t> *to, std::vector<Double_t> *vin)
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

Double_t *AtTPC_Background::TwoB(Double_t m1b, Double_t m2b, Double_t m3b, Double_t m4b, Double_t Kb, Double_t thetacm)
{

   static Double_t kinrec[2];

   double Et1 = Kb + m1b;
   double Et2 = m2b;

   double s = pow(m1b, 2) + pow(m2b, 2) + 2 * m2b * Et1;
   double t = 0.;
   double u = 0.;

   double a = 4. * m2b * s;
   double b = (pow(m3b, 2) - pow(m4b, 2) + s) * (pow(m1b, 2) - pow(m2b, 2) - s);
   double c =
      AtTPC_Background::omega(s, pow(m1b, 2), pow(m2b, 2)) * AtTPC_Background::omega(s, pow(m3b, 2), pow(m4b, 2));

   double Et3 = (c * cos((180. - thetacm) * 3.1415926535 / 180) - b) / a; // estamos viendo el recoil en cm (pi-theta)
   // double Et4 = (Et1 + m2b - Et3);

   double K3 = Et3 - m3b;
   // double K4 = Et4 - m4b;

   //------------------Mandestam variables
   // t = pow(m2b, 2) + pow(m4b, 2) - 2 * m2b * Et4;
   u = pow(m2b, 2) + pow(m3b, 2) - 2 * m2b * Et3;

   double theta_lab = acos(
      ((s - pow(m1b, 2) - pow(m2b, 2)) * (pow(m2b, 2) + pow(m3b, 2) - u) +
       2. * pow(m2b, 2) * (pow(m2b, 2) + pow(m4b, 2) - s - u)) /
      (AtTPC_Background::omega(s, pow(m1b, 2), pow(m2b, 2)) * AtTPC_Background::omega(u, pow(m2b, 2), pow(m3b, 2))));

   kinrec[0] = K3;
   kinrec[1] = theta_lab;

   // std::cout<<K3 <<"  "<<K4<<"  "<<theta_lab<<std::endl;

   return kinrec;
}

std::vector<Double_t> AtTPC_Background::BreakUp(std::vector<Double_t> *Pdeuteron)
{

   static std::vector<double> Pproton(3);

   double mp = 1.0078250322 * 931.494;               // proton
   double mn = 1.0086649158 * 931.494;               // neutron
   double md = mp + mn + 1.0 * (gRandom->Uniform()); // Deuteron unbound around 1 MeV excitation energy
   double pdL[3] = {Pdeuteron->at(0), Pdeuteron->at(1), Pdeuteron->at(2)};
   double EdL = sqrt(pow(pdL[0], 2) + pow(pdL[1], 2) + pow(pdL[2], 2) + pow(md, 2));

   // deuteron breaks up into pn
   double normPdL = sqrt(pow(pdL[0], 2) + pow(pdL[1], 2) + pow(pdL[2], 2));
   double betad = normPdL / EdL;
   double gammad = 1.0 / sqrt(1.0 - pow(betad, 2));
   double S_pn = pow(md, 2);
   double Pcpn = 0.5 * AtTPC_Background::omega(S_pn, pow(mp, 2), pow(mn, 2)) / sqrt(S_pn);

   //-----------generate isotropically theta and phi of particles p and n
   double ran1 = (gRandom->Uniform());
   double ran2 = (gRandom->Uniform());
   double thetapn = acos(2 * ran1 - 1.);
   double phipn = 2 * TMath::Pi() * ran2;

   // generate the p and n in the 2H rest frame
   double pprest[3], pnrest[3];
   pprest[2] = Pcpn * cos(thetapn);
   pprest[0] = Pcpn * sin(thetapn) * cos(phipn);
   pprest[1] = Pcpn * sin(thetapn) * sin(phipn);
   pnrest[2] = -1 * pprest[2];
   pnrest[0] = -1 * pprest[0];
   pnrest[1] = -1 * pprest[1];
   double Eprest = sqrt(pow(Pcpn, 2) + pow(mp, 2));
   double Enrest = sqrt(pow(Pcpn, 2) + pow(mn, 2));

   // boost to 2He frame
   double ppL[3], pnL[3];
   ppL[0] = pprest[0];
   ppL[1] = pprest[1];
   ppL[2] = gammad * (pprest[2] + betad * Eprest);
   // double EpL = sqrt(pow(mp, 2) + pow(ppL[0], 2) + pow(ppL[1], 2) + pow(ppL[2], 2));

   pnL[0] = pnrest[0];
   pnL[1] = pnrest[1];
   pnL[2] = gammad * (pnrest[2] + betad * Enrest);
   // double EnL = sqrt(pow(mn, 2) + pow(pnL[0], 2) + pow(pnL[1], 2) + pow(pnL[2], 2));

   // rotate to the 2H direction
   std::vector<double> fvfrom(3);
   std::vector<double> fvto(3);
   std::vector<double> fvin(3);
   std::vector<double> fvout(3);
   fvfrom.at(0) = 0;
   fvfrom.at(1) = 0;
   fvfrom.at(2) = 1;
   fvto.at(0) = pdL[0];
   fvto.at(1) = pdL[1];
   fvto.at(2) = pdL[2];
   fvin.at(0) = ppL[0];
   fvin.at(1) = ppL[1];
   fvin.at(2) = ppL[2];
   fvout = AtTPC_Background::TRANSF(&fvfrom, &fvto, &fvin);
   Pproton.at(0) = fvout.at(0);
   Pproton.at(1) = fvout.at(1);
   Pproton.at(2) = fvout.at(2);

   // std::cout<<"El breakup*****************  "<<betad<<"  "<<S_pn<<"   "<<Pcpn<<std::endl;

   return Pproton;
}

// -----   Public method ReadEvent   --------------------------------------
Bool_t AtTPC_Background::ReadEvent(FairPrimaryGenerator *primGen)
{

   fIsDecay = kFALSE;

   fBeamEnergy = AtVertexPropagator::Instance()->GetEnergy();
   std::cout << " -I- AtTPC_Background Residual energy  : " << AtVertexPropagator::Instance()->GetEnergy() << std::endl;

   fPxBeam = AtVertexPropagator::Instance()->GetPx();
   fPyBeam = AtVertexPropagator::Instance()->GetPy();
   fPzBeam = AtVertexPropagator::Instance()->GetPz();

   // fPxBeam = fPx.at(0) ;
   // fPyBeam = fPy.at(0) ;
   // fPzBeam = fPz.at(0) ;

   if (fBeamEnergy == 0) {
      std::cout << "-I- AtTP_Background : No solution!" << std::endl;
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
   }

   m1 = Masses.at(0) * amu + fExEnergy.at(0);
   m2 = Masses.at(1) * amu + fExEnergy.at(1);
   m3 = Masses.at(2) * amu;                   // recoil 1
   m4 = Masses.at(3) * amu + fExEnergy.at(3); // ejectile 1
   m7 = Masses.at(4) * amu;                   // recoil 2
   m8 = Masses.at(5) * amu + fExEnergy.at(5); // ejectile 2
   K1 = sqrt(pow(fPx.at(0), 2) + pow(fPy.at(0), 2) + pow(fPz.at(0), 2) + pow(m1, 2)) - m1;
   // K1 = sqrt(pow(fPxBeam*1000.0,2) + pow(fPyBeam*1000.0,2) + pow( fPzBeam*1000.0,2) + pow(m1,2)) - m1;

   Double_t fThetaCmsMin = 0.;
   Double_t fThetaCmsMax = 8;

   Double_t costhetamin = TMath::Cos(fThetaCmsMin * TMath::DegToRad());
   Double_t costhetamax = TMath::Cos(fThetaCmsMax * TMath::DegToRad());

   /*
   //proton 1 from (d,p)
   ////uniform thetacm distribution between thetamin and thetamax
   Double_t thetacmsInput = TMath::ACos( (costhetamax - costhetamin )*gRandom->Uniform() + costhetamin
)*TMath::RadToDeg(); Double_t* kin2B1 = AtTPC_Background::TwoB(m1, m2, m3, m4, K1, thetacmsInput); Double_t phi1 =
2*TMath::Pi() * gRandom->Uniform();         //flat probability in phi Double_t krec = *(kin2B1+0); Double_t angrec =
*(kin2B1+1); Prec = sqrt( pow(krec,2) + 2*krec*m3); fPx.at(2) = (Prec*sin(angrec)*cos(phi1) )/1000.0; // To GeV for
FairRoot fPy.at(2) = (Prec*sin(angrec)*sin(phi1) )/1000.0; // To GeV for FairRoot fPz.at(2) = (Prec*cos(angrec)
)/1000.0; // To GeV for FairRoot
   //std::cout<<"Kin 1  "<< angrec<<"  "<<krec<<std::endl;



   //proton 2 from (d,p)
   ////uniform thetacm distribution between thetamin and thetamax
   thetacmsInput = TMath::ACos( (costhetamax - costhetamin )*gRandom->Uniform() + costhetamin )*TMath::RadToDeg();
   Double_t* kin2B2 = AtTPC_Background::TwoB(m1, m2, m3, m4, K1, thetacmsInput);
   Double_t phi2 = 2*TMath::Pi() * gRandom->Uniform();         //flat probability in phi
   krec = *(kin2B2+0);
   angrec = *(kin2B2+1);
   Prec = sqrt( pow(krec,2) + 2*krec*m3);
   fPx.at(3) = (Prec*sin(angrec)*cos(phi2) )/1000.0; // To GeV for FairRoot
fPy.at(3) = (Prec*sin(angrec)*sin(phi2) )/1000.0; // To GeV for FairRoot
fPz.at(3) = (Prec*cos(angrec) )/1000.0; // To GeV for FairRoot
   //std::cout<<"Kin 2  "<< angrec<<"  "<<krec<<std::endl;
  */

   // proton 1 from breakup
   ////uniform thetacm distribution between thetamin and thetamax
   Double_t thetacmsInput =
      TMath::ACos((costhetamax - costhetamin) * gRandom->Uniform() + costhetamin) * TMath::RadToDeg();
   Double_t *kin2B1 = AtTPC_Background::TwoB(m1, m2, m7, m8, K1, thetacmsInput);
   Double_t phi1 = 2 * TMath::Pi() * gRandom->Uniform(); // flat probability in phi
   Double_t krec = *(kin2B1 + 0);
   Double_t angrec = *(kin2B1 + 1);
   Prec = sqrt(pow(krec, 2) + 2 * krec * m2);
   std::vector<double> Pdeut(3);
   Pdeut.at(0) = (Prec * sin(angrec) * cos(phi1));
   Pdeut.at(1) = (Prec * sin(angrec) * sin(phi1));
   Pdeut.at(2) = (Prec * cos(angrec));
   std::vector<double> Pprot1 = AtTPC_Background::BreakUp(&Pdeut);
   fPx.at(2) = (Pprot1.at(0)) / 1000.0; // To GeV for FairRoot
   fPy.at(2) = (Pprot1.at(1)) / 1000.0; // To GeV for FairRoot
   fPz.at(2) = (Pprot1.at(2)) / 1000.0; // To GeV for FairRoot
   // std::cout<<"Kin 1  "<< acos(Pprot1.at(2)/( sqrt(  pow(Pprot1.at(0),2) + pow(Pprot1.at(1),2) + pow(Pprot1.at(2),2))
   // ))*180.0/3.1415<<"  "<<sqrt(  pow(Pprot1.at(0),2) + pow(Pprot1.at(1),2) + pow(Pprot1.at(2),2) + pow(m3,2) )
   // -m3<<std::endl;

   // proton 2 from breakup
   ////uniform thetacm distribution between thetamin and thetamax
   thetacmsInput = TMath::ACos((costhetamax - costhetamin) * gRandom->Uniform() + costhetamin) * TMath::RadToDeg();
   Double_t *kin2B2 = AtTPC_Background::TwoB(m1, m2, m7, m8, K1, thetacmsInput);
   Double_t phi2 = 2 * TMath::Pi() * gRandom->Uniform(); // flat probability in phi
   krec = *(kin2B2 + 0);
   angrec = *(kin2B2 + 1);
   Prec = sqrt(pow(krec, 2) + 2 * krec * m2);
   Pdeut.at(0) = (Prec * sin(angrec) * cos(phi2));
   Pdeut.at(1) = (Prec * sin(angrec) * sin(phi2));
   Pdeut.at(2) = (Prec * cos(angrec));
   std::vector<double> Pprot2 = AtTPC_Background::BreakUp(&Pdeut);
   fPx.at(3) = (Pprot2.at(0)) / 1000.0; // To GeV for FairRoot
   fPy.at(3) = (Pprot2.at(1)) / 1000.0; // To GeV for FairRoot
   fPz.at(3) = (Pprot2.at(2)) / 1000.0; // To GeV for FairRoot
   // std::cout<<"Kin 2  "<< acos(Pprot2.at(2)/( sqrt(  pow(Pprot2.at(0),2) + pow(Pprot2.at(1),2) + pow(Pprot2.at(2),2))
   // ))*180.0/3.1415<<"  "<<sqrt(  pow(Pprot2.at(0),2) + pow(Pprot2.at(1),2) + pow(Pprot2.at(2),2) + pow(m3,2) )
   // -m3<<std::endl;

   // std::cout<<"Kin 2  "<< angrec<<"  "<<krec<<std::endl;

   // proton 3 from breakup
   ////uniform thetacm distribution between thetamin and thetamax
   thetacmsInput = TMath::ACos((costhetamax - costhetamin) * gRandom->Uniform() + costhetamin) * TMath::RadToDeg();
   Double_t *kin2B3 = AtTPC_Background::TwoB(m1, m2, m7, m8, K1, thetacmsInput);
   Double_t phi3 = 2 * TMath::Pi() * gRandom->Uniform(); // flat probability in phi
   krec = *(kin2B3 + 0);
   angrec = *(kin2B3 + 1);
   Prec = sqrt(pow(krec, 2) + 2 * krec * m2);
   Pdeut.at(0) = (Prec * sin(angrec) * cos(phi3));
   Pdeut.at(1) = (Prec * sin(angrec) * sin(phi3));
   Pdeut.at(2) = (Prec * cos(angrec));
   std::vector<double> Pprot3 = AtTPC_Background::BreakUp(&Pdeut);
   fPx.at(4) = (Pprot3.at(0)) / 1000.0; // To GeV for FairRoot
   fPy.at(4) = (Pprot3.at(1)) / 1000.0; // To GeV for FairRoot
   fPz.at(4) = (Pprot3.at(2)) / 1000.0; // To GeV for FairRoot
   // std::cout<<"Kin 3  "<< acos(Pprot3.at(2)/( sqrt(  pow(Pprot3.at(0),2) + pow(Pprot3.at(1),2) + pow(Pprot3.at(2),2))
   // ))*180.0/3.1415<<"  "<<sqrt(  pow(Pprot3.at(0),2) + pow(Pprot3.at(1),2) + pow(Pprot3.at(2),2) + pow(m3,2) )
   // -m3<<std::endl;

   // proton 4 from breakup
   ////uniform thetacm distribution between thetamin and thetamax
   thetacmsInput = TMath::ACos((costhetamax - costhetamin) * gRandom->Uniform() + costhetamin) * TMath::RadToDeg();
   Double_t *kin2B4 = AtTPC_Background::TwoB(m1, m2, m7, m8, K1, thetacmsInput);
   Double_t phi4 = 2 * TMath::Pi() * gRandom->Uniform(); // flat probability in phi
   krec = *(kin2B4 + 0);
   angrec = *(kin2B4 + 1);
   Prec = sqrt(pow(krec, 2) + 2 * krec * m2);
   Pdeut.at(0) = (Prec * sin(angrec) * cos(phi4));
   Pdeut.at(1) = (Prec * sin(angrec) * sin(phi4));
   Pdeut.at(2) = (Prec * cos(angrec));
   std::vector<double> Pprot4 = AtTPC_Background::BreakUp(&Pdeut);
   fPx.at(5) = (Pprot4.at(0)) / 1000.0; // To GeV for FairRoot
   fPy.at(5) = (Pprot4.at(1)) / 1000.0; // To GeV for FairRoot
   fPz.at(5) = (Pprot4.at(2)) / 1000.0; // To GeV for FairRoot
   // std::cout<<"Kin 4  "<< acos(Pprot4.at(2)/( sqrt(  pow(Pprot4.at(0),2) + pow(Pprot4.at(1),2) + pow(Pprot4.at(2),2))
   // ))*180.0/3.1415<<"  "<<sqrt(  pow(Pprot4.at(0),2) + pow(Pprot4.at(1),2) + pow(Pprot4.at(2),2) + pow(m3,2) )
   // -m3<<std::endl;

   do {
      // random_z = 100.0*(gRandom->Uniform()); //cm
      random_r = 1.0 * (gRandom->Gaus(0, 1));                // cm
      random_phi = 2.0 * TMath::Pi() * (gRandom->Uniform()); // rad

   } while (fabs(random_r) > 4.7); // cut at 2 sigma

   for (Int_t i = 0; i < fMult; i++) {

      int pdgType = 2212;

      fVx = random_r * cos(random_phi);
      fVy = random_r * sin(random_phi);
      fVz = 100.0 * (gRandom->Uniform()); // cm

      if (i > 1 && AtVertexPropagator::Instance()->GetDecayEvtCnt() && pdgType == 2212) {
         // TODO: Dirty way to propagate only the products (0 and 1 are beam and target respectively)

         // std::cout << "-I- FairIonGenerator: Generating ions of type "
         //<< fIon.at(i)->GetName() << " (PDG code " << pdgType << ")" << std::endl;
         std::cout << "    Momentum (" << fPx.at(i) << ", " << fPy.at(i) << ", " << fPz.at(i) << ") Gev from vertex ("
                   << fVx << ", " << fVy << ", " << fVz << ") cm" << std::endl;

         primGen->AddTrack(pdgType, fPx.at(i), fPy.at(i), fPz.at(i), fVx, fVy, fVz);
      }
   }

   AtVertexPropagator::Instance()
      ->IncDecayEvtCnt(); // TODO: Okay someone should put a more suitable name but we are on a hurry...

   return kTRUE;
}

ClassImp(AtTPC_Background)
