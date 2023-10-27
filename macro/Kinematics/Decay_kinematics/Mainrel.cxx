#include <Riostream.h>
#include <TApplication.h>
#include <TAxis.h>
#include <TBranch.h>
#include <TFile.h>
#include <TGraph.h>
#include <TH1F.h>
#include <TMath.h>
#include <TROOT.h>
#include <TRint.h>
#include <TTree.h>

#include "TRelativisticKinematics.hh"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "TRelativisticKinematics.cxx"
//#include "TRelativisticDecay.hh"
//#include "TRelativisticDecay.cxx"

Double_t read_ame03(int Zin, int Ain, char *El);

using namespace std;

int Mainrel()
{

   std::ofstream kineStr;
   kineStr.open("Kine.txt");

   Int_t a = 1;
   int A1 = 16.; // mass0 of the incident particle
   int Z1 = 6.;  // charge of the incident particle
   char *El1 = new char[2];

   // double m1=77.963180;

   // cout<<El1<<endl;

   int A2 = 1.; // mass0 of the target
   int Z2 = 1.; // charge of the target
   char *El2 = new char[2];

   // double m2=2.0141;

   // cout<<El2<<endl;

   int A3 = 16; // mass0 of the scattered particle
   int Z3 = 6;  // charge of the scattered particle
   char *El3 = new char[2];

   // double m3=78.971987;

   // cout<<El3<<endl;

   int A4 = 1; // mass0 of the recoil
   int Z4 = 1; // charge of the recoil
   char *El4 = new char[2];

   // double m4=1.007825;

   // cout<<El4<<endl;

   int AD1 = 12; // mass0 of the daughter 1
   int ZD1 = 2;  // charge of the daugther 2
   char *ElD1 = new char[2];

   int AD2 = 4; // mass0 of the daughter 1
   int ZD2 = 2; // charge of the daugther 2
   char *ElD2 = new char[2];

   double ex1 = 0;     // excitation energy of the incident particle
   double ex2 = 0;     // excitation energy of the target
   double ex3 = 3.027; // excitation energy of the scattered particle
   double ex4 = 0.0;   // excitation energy of the recoil

   double tbt = 12.0; // incident energy (total Lab energy in MeV)

   double Energyrdec, Anglerdec;

   //  cout<<" Mass of the incident particle A1"<<endl;
   //  cin>>A1;
   //  cout<<" Charge of the incident particle Z1"<<endl;
   //  cin>>Z1;
   double m1 = read_ame03(Z1, A1, El1);
   std::cout << " m1 " << m1 << "\n";
   // cout<<" Mass of the target A2"<<endl;
   // cin>>A2;
   // cout<<" Charge of the target Z2"<<endl;
   // cin>>Z2;
   double m2 = read_ame03(Z2, A2, El2);
   std::cout << " m2 " << m2 << "\n";
   // cout<<" Mass of the scattered particle A3"<<endl;
   // cin>>A3;
   // cout<<" Charge of the scattered particle Z3"<<endl;
   // cin>>Z3;
   double m3 = read_ame03(Z3, A3, El3);
   std::cout << " m3 " << m3 << "\n";
   // cout<<" Mass of the recoil particle A4"<<endl;
   // cin>>A4;
   // cout<<" Charge of the recoil particle Z4"<<endl;
   // cin>>Z4;
   double m4 = read_ame03(Z4, A4, El4);
   std::cout << " m4 " << m4 << "\n";
   // cout<<" Excitation energy of the recoil "<<endl;
   // cin>>ex4;
   // double md1=read_ame03(ZD1,AD1,ElD1);
   // double md2=read_ame03(ZD2,AD2,ElD2);

   double tb = tbt * m1;
   Double_t ThetaCM[20000];
   Double_t Energyr[20000];
   Double_t Angler[20000];
   Double_t Energys[20000];
   Double_t Angles[20000];
   Double_t AngleD1[20000];
   Double_t AngleD2[20000];
   Double_t EnergyD1[20000];
   Double_t EnergyD2[20000];
   Double_t ThetaCMdec[20000];
   Int_t j = 0, aux = 0;
   // double thetacmsInput=12.34; // Theta CM angle of the scattered particle, in degrees

   // double thetacmsInput=atof(argv[1]); // Theta CM angle of the scattered particle, in degrees

   // TApplication theApp("App", &argc, argv);
   // TRint theApp("App", &argc, argv);

   TRelativisticKinematics *kine = new TRelativisticKinematics;
   // TRelativisticDecay *dec=new TRelativisticDecay;

   TH2F *h0 = new TH2F("h0", "", 1000, 0, 100, 1000, 0, 100);
   TH2F *h1 = new TH2F("h1", "", 1000, 0, 100, 1000, 0, 100);
   TH2F *h2 = new TH2F("h2", "", 1000, 0, 100, 1000, 0, 100);

   kine->SetMassOfProjectile(m1);
   kine->SetExEnergyOfProjectile(ex1);
   kine->SetMassOfTarget(m2);
   kine->SetExEnergyOfTarget(ex2);
   kine->SetMassOfScattered(m3);
   kine->SetExEnergyOfScattered(ex3);
   kine->SetMassOfRecoiled(m4);
   kine->SetExEnergyOfRecoiled(ex4);
   kine->SetLabEnergy(tb);

   cout << "ThetaCM(deg) ThetaR(deg)   EnergyR(Mev)" << endl;
   for (Float_t k = 0; k <= 180; k = k + 0.1) {
      // for(Float_t k=0;k<=180;k=k+2){
      kine->SetThetaCMAngle(k);
      kine->Kinematics();
      // kine->PrintResults();
      Angler[j] = kine->GetANGAr(0) * 180 / TMath::Pi();
      Energyr[j] = kine->GetANGAr(1);
      //    if((k-TMath::Floor(k)==0))
      //  cout<<k<<" "<<Angler[j]<<" "<<Energyr[j]<<endl;
      Angles[j] = kine->GetANGAs(0) * 180 / TMath::Pi();
      Energys[j] = kine->GetANGAs(1);
      ThetaCM[j] = k;
      kineStr << ThetaCM[j] << " " << Angler[j] << " " << Energyr[j] << " " << Angles[j] << " " << Energys[j]
              << std::endl;
      j++;
   }

   TGraph *g2 = new TGraph(j, Angler, Energyr);
   TGraph *g3 = new TGraph(j, ThetaCM, Angler);
   TGraph *g = new TGraph(j, Angles, Energys);
   TGraph *g3p = new TGraph(j, Angles, Angler);

   Char_t grafTitle1[256], grafTitle2[256], grafTitle3[256], grafTitle3p[256], grafTitle4[256], grafTitle5[256],
      grafTitle6[256], grafTitle6p[256];

   sprintf(grafTitle1, "Kinematics for ^{%d}%s(^{%d}%s,^{%d}%s)^{%d}%s at %g AMeV (Scatter)", A1, El1, A2, El2, A4, El4,
           A3, El3, tbt);

   sprintf(grafTitle2, "Kinematics for ^{%d}%s(^{%d}%s,^{%d}%s)^{%d}%s at %g AMeV (Recoil)", A1, El1, A2, El2, A4, El4,
           A3, El3, tbt);

   sprintf(grafTitle3, "Kinematics for ^{%d}%s(^{%d}%s,^{%d}%s)^{%d}%s at %g AMeV (Recoil)", A1, El1, A2, El2, A4, El4,
           A3, El3, tbt);

   sprintf(grafTitle3p, "Kinematics for ^{%d}%s(^{%d}%s,^{%d}%s)^{%d}%s at %g AMeV (Recoil vs Scatter)", A1, El1, A2,
           El2, A4, El4, A3, El3, tbt);

   g->SetTitle(grafTitle1);
   g->GetXaxis()->SetTitle("#theta_{lab} (deg)");
   g->GetYaxis()->SetTitle("Energy (MeV)");
   g2->SetTitle(grafTitle2);
   g2->GetXaxis()->SetTitle("#theta_{lab} (deg)");
   g2->GetYaxis()->SetTitle("Energy (MeV)");
   g3->SetTitle(grafTitle3);
   g3->GetXaxis()->SetTitle("#theta_{CM} (deg)");
   g3->GetYaxis()->SetTitle("#theta_{Lab} (deg)");
   g3p->SetTitle(grafTitle3p);
   g3p->GetXaxis()->SetTitle("#theta_{Lab} Scatter (deg)");
   g3p->GetYaxis()->SetTitle("#theta_{Lab} Recoil (deg)");

   TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->Draw();
   c1->cd(1);
   g2->Draw("AC");
   c1->cd(2);
   g->Draw("AC");
   c1->cd(3);
   g3->Draw("AC");
   c1->cd(4);
   g3p->Draw("AC");

   /*TCanvas* c2=new TCanvas();
   c2->Divide(2,2);
   c2->Draw();
   c2->cd(1);
   g4->Draw("AC");
   c2->cd(2);
   g5->Draw("AC");
   c2->cd(3);
   g6->Draw("AC");
   c2->cd(4);
   g6p->Draw("AC");*/

   /*TCanvas* c4=new TCanvas();
    h0->SetTitle(" Angle Lab ^{12}Be - ^{4}He ");
   h0->GetXaxis()->SetTitle("#theta_{Lab} ^{4}He (deg)");
   h0->GetYaxis()->SetTitle("#theta_{Lab} ^{12}Be (deg)");
   h0->Draw("COL");

   TCanvas* c3=new TCanvas();
    h1->SetTitle(" Angle-Energy Lab ^{4}He ");
   h1->GetXaxis()->SetTitle("#theta_{Lab} ^{4}He (deg)");
   h1->GetYaxis()->SetTitle("Energy ^{4}He (MeV)");
   h1->Draw("COL");

   TCanvas* c2=new TCanvas();
    h2->SetTitle(" Angle Lab ^{12}Be ");
   h2->GetXaxis()->SetTitle("#theta_{Lab} ^{12}Be (deg)");
   h2->GetYaxis()->SetTitle("Energy ^{12}Be (MeV)");
   h2->Draw("COL");*/

   // delete g;
   // kine->PrintResults();
   delete kine;
   //	delete dec;
   return 0;
}

Double_t read_ame03(int Zin, int Ain, char *El)
{
   // std::ifstream *in = new std::ifstream("masstable.dat");
   std::ifstream in("masstable.dat");
   Char_t line[256];
   Int_t N;
   Int_t Z;
   Int_t A;
   Char_t elem[2];
   Char_t dummy[256];
   Double_t mass;
   for (Int_t k = 0; k < 39; k++) {
      in.getline(line, 256);
      // cout<<line<<endl;
   }
   while (!in.eof()) {
      in >> N >> Z >> elem;
      // cout<<N<<" "<<Z<<" "<<elem<<endl;
      in.get(line, 256, '\n');
      // cout<<line<<endl;
      Int_t a = atoi(elem);
      if (a != 0) {
         sscanf(line, "%s %lf %*lf", elem, &mass);
         A = a;
      } else {
         sscanf(line, " %lf %*lf ", &mass);
         A = N + Z;
      }

      if (Z == Zin && A == Ain) {
         // cout<<A<<elem<<" "<<mass/1e6<<endl;
         //  if(Z==1){
         //    if(A==1)
         //      strcpy(El,"p");
         //    else if(A==2)
         //      strcpy(El,"d");
         //    else if(A==3)
         //      strcpy(El,"t");
         //  }
         //  else
         strcpy(El, elem);
         return mass / 1e6;
      }
   }
   cout << "Mass not found!" << endl;
   // delete in;
   return -1;
}
