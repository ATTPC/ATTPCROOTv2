#include "TString.h"
#include "TFile.h"
#include "TTree.h"
#include "TClonesArray.h"

#include <iostream>
#include <fstream>

Double_t Gauss(Double_t mu, Double_t sigma)
{

   std::random_device rd;
   std::mt19937 e2(rd());
   std::normal_distribution<> dist(mu, sigma);

   return dist(e2);
}

Double_t omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

double kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject)
{

   // in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
   double Et1 = K_proj + m1;
   double Et2 = m2;
   double Et3 = K_eject + m3;
   double Et4 = Et1 + Et2 - Et3;
   double m4_ex, Ex, theta_cm;
   double s, t, u; //---Mandelstam variables

   s = pow(m1, 2) + pow(m2, 2) + 2 * m2 * Et1;
   u = pow(m2, 2) + pow(m3, 2) - 2 * m2 * Et3;

   m4_ex = sqrt((cos(thetalab) * omega(s, pow(m1, 2), pow(m2, 2)) * omega(u, pow(m2, 2), pow(m3, 2)) -
                 (s - pow(m1, 2) - pow(m2, 2)) * (pow(m2, 2) + pow(m3, 2) - u)) /
                   (2 * pow(m2, 2)) +
                s + u - pow(m2, 2));
   Ex = m4_ex - m4;

   t = pow(m2, 2) + pow(m4_ex, 2) - 2 * m2 * Et4;

   // for inverse kinematics Note: this angle corresponds to the recoil
   theta_cm = TMath::Pi() - acos((pow(s, 2) + s * (2 * t - pow(m1, 2) - pow(m2, 2) - pow(m3, 2) - pow(m4_ex, 2)) +
                                  (pow(m1, 2) - pow(m2, 2)) * (pow(m3, 2) - pow(m4_ex, 2))) /
                                 (omega(s, pow(m1, 2), pow(m2, 2)) * omega(s, pow(m3, 2), pow(m4_ex, 2))));

   // THcm = theta_cm*TMath::RadToDeg();
   return Ex;
}

void sega_ana(Int_t num_ev = 2000)
{

   TH2D *Eloss_vs_Range_Sca = new TH2D("Eloss_vs_Range_Sca", "ELoss_vs_Range_Sca", 100, 0, 1000, 300, 0, 300);
   Eloss_vs_Range_Sca->SetMarkerStyle(20);
   Eloss_vs_Range_Sca->SetMarkerSize(0.5);

   TH2D *Eloss_vs_Range_Rec = new TH2D("Eloss_vs_Range_Rec", "ELoss_vs_Range_Rec", 1000, 0, 2000, 1000, 0, 10);
   Eloss_vs_Range_Rec->SetMarkerStyle(20);
   Eloss_vs_Range_Rec->SetMarkerSize(0.5);
   Eloss_vs_Range_Rec->SetMarkerColor(2);

   TH2D *tracks = new TH2D("tracks", "tracks", 1000, -100, 1000, 1000, -300, 300);

   TH1D *ELossRatio = new TH1D("ElossRatio", "ELossRatio", 1000, 0, 1000);

   TH1D *ICELoss = new TH1D("ICELoss", "ICELoss", 1000, 0, 100);

   TH1D *rad = new TH1D("rad", "rad", 2000, 0, 2000);

   TH2D *HKineRecoil = new TH2D("HKineRecoil", "HKineRecoil", 1000, 0, 180, 1000, 0, 200);
   TH2D *AngRange[10];
   Char_t hh_name[10][256];

   for (Int_t i = 0; i < 10; i++) {
      std::sprintf(hh_name[i], "angrange_%d", i);
      AngRange[i] = new TH2D(hh_name[i], hh_name[i], 100, 0, 180, 100, 0, 2000);
   }

   TH2D *RadAng[10];
   TH2D *Range_vs_Energy_Sca = new TH2D("Range_vs_Energy_Sca", "Range_vs_Energy_Sca", 200, 0, 1000, 500, 0, 50);
   TH2D *Range_vs_Energy_Rec = new TH2D("Range_vs_Energy_Rec", "Range_vs_Energy_Rec", 200, 0, 1000, 500, 0, 50);
   TH2D *Range_vs_Energy_Beam = new TH2D("Range_vs_Energy_Beam", "Range_vs_Energy_Beam", 200, 0, 1000, 500, 0, 50);

   TH3D *Vertex_vs_Angle_Rec =
      new TH3D("Vertex_vs_Angle_Rec", "Vertex_vs_Angle_Rec", 100, 0, 500, 100, 0, 500, 100, 0, 180);

   TH2D *Angle_sca_vs_Angle_rec =
      new TH2D("Angle_sca_vs_Angle_rec", "Angle_sca_vs_Angle_rec", 200, 0, 180, 200, 0, 180);
   Char_t hra_name[10][256];

   for (Int_t i = 0; i < 10; i++) {
      std::sprintf(hra_name[i], "radang_%d", i);
      RadAng[i] = new TH2D(hra_name[i], hra_name[i], 100, 0, 180, 100, 0, 200);
   }

   TH1F *HQval = new TH1F("HQval", "HQval", 1000, -10, 10);

   TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->Draw();

   TCanvas *c2 = new TCanvas();
   c2->Divide(2, 2);
   c2->Draw();

   /*  TCanvas *c3 = new TCanvas();
     c3->Divide(3,3);
     c3->Draw();
     TCanvas *c4 = new TCanvas();
     c4->Divide(3,3);
     c4->Draw();
     TCanvas *c5 = new TCanvas();
     c5->Divide(2,2);
     c5->Draw();
     TCanvas *c6 = new TCanvas();
     c6->Divide(1,3);
     c6-> Draw();
     TCanvas *c7 = new TCanvas();
     c7-> Draw();*/

   std::vector<TString> files = {"data/SeGA_el", "data/SeGA_02_el", "data/SeGA_02_1"};

   std::vector<double> ex_energy_beam = {0.0, 2.251, 2.251};
   std::vector<double> ex_energy = {0.0, 2.251, 2.715};

   std::vector<int> xs_events = {20000, 20000, 20000};

   for (auto iFile = 0; iFile < files.size(); ++iFile) {

      TString mcFileNameHead = files[iFile];
      TString mcFileNameTail = ".root";
      TString mcFileName = mcFileNameHead + mcFileNameTail;
   std:
      cout << " Analysis of simulation file  " << mcFileName << endl;

      AtGeArrayPoint *point = new AtGeArrayPoint();
      AtGeArrayPoint *point_forw = new AtGeArrayPoint();
      AtGeArrayPoint *point_back = new AtGeArrayPoint();
      TClonesArray *pointArray = 0;
      TFile *file = new TFile(mcFileName.Data(), "READ");
      TTree *tree = (TTree *)file->Get("cbmsim");

      tree = (TTree *)file->Get("cbmsim");
      // TBranch *branch = tree->GetBranch("AtTpcPoint");
      tree->SetBranchAddress("AtGeArrayPoint", &pointArray);
      Int_t nEvents = tree->GetEntriesFast();

      Double_t vertex = 0.0;
      Double_t vertex_buff = 0.0;
      Double_t Ebeam_buff = 0.0;

      if (nEvents > num_ev)
         nEvents = num_ev;

      for (Int_t iEvent = 0; iEvent < nEvents; iEvent++) {
         Double_t energyLoss_sca = 0.0;
         Double_t range_sca = 0.0;
         Double_t energyLoss_rec = 0.0;
         Double_t range_rec = 0.0;
         Double_t range_beam = 0.0;
         Double_t BeamEnergyLoss_IC = 0.0;
         Double_t EnergyRecoil = 0.0;
         Double_t EnergyBeam = 0.0;
         Double_t EnergySca = 0.0;
         Double_t AngleRecoil = 0.0;
         Double_t AngleSca = 0.0;
         Double_t zpos = 0.0;
         Double_t xpos = 0.0;
         Double_t radius = 0.0;
         Double_t radmean = 0.0;
         Double_t thetamean = 0.0;
         Double_t theta = 0.0;
         Int_t n2 = 0;
         Int_t nrad = 0;

         if (iEvent % 2 == 0) {
            vertex_buff = 0.0;
            Ebeam_buff = 0.0;
         }

         // std::cout<<" Event - Vertex "<<iEvent<<"	"<<vertex_buff<<"\n";

         TString VolName;
         tree->GetEvent(iEvent);
         // tree -> GetEntry(iEvent);
         Int_t n = pointArray->GetEntries();
         if (iEvent % 100 == 0)
            std::cout << " Event Number : " << iEvent << std::endl;
         rad->Reset();

         for (Int_t i = 0; i < n; i++) {

            point = (AtGeArrayPoint *)pointArray->At(i);
            VolName = point->GetVolName();
            // std::cout<<" Volume Name : "<<VolName<<std::endl;
            Int_t trackID = point->GetTrackID();

            // if(i==0){
            //	zpos=point->GetZ();
            // std::cout<<" Z pos : "<<zpos<<std::endl;
            // std::cout<<" Track ID : "<<trackID<<std::endl;
            //   }
            // std::cout<<" Track ID : "<<trackID<<std::endl;
            // std::cout<<" Point number : "<<i<<std::endl;
            if (trackID == 0 && VolName.Contains("IC_")) {

               BeamEnergyLoss_IC += (point->GetEnergyLoss()) * 1000; // MeV
            }

            if (trackID == 0 && VolName == "germanium") {
               vertex = point->GetZ() * 10;
               range_beam = point->GetLength() * 10; // mm
               EnergyBeam = point->GetEIni();
               // std::cout<<" Vertex : "<<vertex<<std::endl;
               vertex_buff = vertex;
               BeamEnergyLoss_IC += (point->GetEnergyLoss()) * 1000; // MeV
            }

            if (trackID == 2 && VolName == "germanium") { // RECOIL
               n2++;
               range_rec = point->GetLength() * 10;               // mm
               energyLoss_rec += (point->GetEnergyLoss()) * 1000; // MeV
               EnergyRecoil = point->GetEIni();
               AngleRecoil = point->GetAIni();
               zpos = point->GetZ() * 10;
               xpos = point->GetXIn() * 10;
               // if(AngleRecoil>65 && AngleRecoil<70 && vertex>200.0 && vertex<210.0 && zpos>200.0 ){
               tracks->Fill(zpos, xpos);
               //}

               // Radius of curvature calculation
               // if(i<(n-2)){
               if (i < 5) {
                  nrad++;
                  point_forw = (AtGeArrayPoint *)pointArray->At(i + 1);
                  point_back = (AtGeArrayPoint *)pointArray->At(i + 2);
                  Double_t x1 = point->GetXIn() * 10;
                  Double_t x2 = point_forw->GetXIn() * 10;
                  Double_t x3 = point_back->GetXIn() * 10;
                  Double_t y1 = point->GetYIn() * 10;
                  Double_t y2 = point_forw->GetYIn() * 10;
                  Double_t y3 = point_back->GetYIn() * 10;
                  Double_t z2 = point_forw->GetZIn() * 10;

                  Double_t dl2 = pow((x1 - x3), 2) + pow((y1 - y3), 2);
                  Double_t dr1 = pow(x2 - (x3 + x1) / 2.0, 2);
                  Double_t dr2 = pow(y2 - (y3 + y1) / 2.0, 2);
                  Double_t dr = TMath::Sqrt(dr1 + dr2) * 8.0;
                  radius = dl2 / dr;
                  Double_t dlt = TMath::Sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
                  Double_t dy = y2 - y1;
                  Double_t dx = x2 - x1;

                  theta = TMath::ATan2(dlt, zpos - z2);
                  // std::cout<<zpos-z2<<std::endl;
                  // Double_t phi = TMath::ATan2(dy,dx);
                  /*std::cout<<" ======================= "<<std::endl;
                  std::cout<<" Point : "<<i<<" of "<<n<<std::endl;
                  std::cout<<180-theta*180/TMath::Pi()<<std::endl;
                  std::cout<<AngleRecoil<<std::endl;*/
                  rad->SetBinContent(i + 1, radius);
                  radmean += radius;
                  thetamean += theta;
                  // std::cout<<" Point : "<<i<<" radius : "<<radius<<std::endl;
               } // Radius calculation

               //}

               // std::cout<<point->GetEIni()<<std::endl;
               // std::cout<<" Track ID : "<<trackID<<std::endl;

               // std::cout<<" Point number : "<<i<<std::endl;
               // std::cout<<" Event Number : "<<iEvent<<std::endl;
               // std::cout<<" Range_rec : "<<range_rec<<std::endl;
               // std::cout<<" energyLoss_rec : "<<energyLoss_rec<<std::endl;

            }                                                // TrackID == 2
            if (trackID == 1 && VolName == "germanium") { // SCATTER
               range_sca = point->GetLength() * 10;          // mm
               EnergySca = point->GetEIni();
               energyLoss_sca += (point->GetEnergyLoss()) * 1000; // MeV
               AngleSca = point->GetAIni();
               // std::cout<<" Track ID : "<<trackID<<std::endl;
               // std::cout<<" Range_sca : "<<range_sca<<std::endl;
               // std::cout<<" energyLoss_sca : "<<energyLoss_sca<<std::endl;
            } // TrackID == 1

         } // n number of points

         // radmean=rad->GetMean();

         radmean = radmean / nrad;
         thetamean = thetamean / nrad;
         // std::cout<<" Theta mean : "<<thetamean*180/TMath::Pi()<<std::endl;
         //  std::cout<<" Theta sim : "<<AngleRecoil<<std::endl;

         // std::cout<<" Mean radius of curvature : "<<radmean<<std::endl;
         // std::cout<<" Mean radius of curvature (hist) : "<<rad->GetMean()<<std::endl;

         if (iEvent % 2 != 0) {
            // std::cout<<" Range_rec : "<<range_rec<<std::endl;
            // std::cout<<" energyLoss_rec : "<<energyLoss_rec<<std::endl;
            Eloss_vs_Range_Rec->Fill(range_rec, energyLoss_rec);

            // std::cout<<" Range_sca : "<<range_sca<<std::endl;
            // std::cout<<" energyLoss_sca : "<<energyLoss_sca<<std::endl;
            Eloss_vs_Range_Sca->Fill(range_sca, energyLoss_sca);
            ELossRatio->Fill(energyLoss_sca / energyLoss_rec);

            // Resolution
            Double_t AngleRecoil_res = Gauss(AngleRecoil, 0.300);
            Double_t EnergyRecoil_res = Gauss(EnergyRecoil, 0.250);

            // Q-value calculation
            Double_t m_a = 1.0078250322 * 931.49401;
            Double_t m_P35 = 12.026922 * 931.49401; // 35P
            Double_t m_beam = 12.026922 * 931.49401 + ex_energy_beam[iFile];
            Ebeam_buff = (EnergyRecoil + EnergySca + ex_energy[iFile]);
            Double_t Qval =
               EnergyRecoil * (1 + m_a / m_P35) - Ebeam_buff * (1 - m_beam / m_P35) -
               (2.0 / m_P35) * TMath::Sqrt(Ebeam_buff * EnergyRecoil_res * m_a) * TMath::Cos(AngleRecoil_res);

            // Excitation energy
            Double_t ex_energy_exp =
               kine_2b(m_P35, m_a, m_a, m_P35, Ebeam_buff, TMath::DegToRad() * AngleRecoil_res, EnergyRecoil_res);
            // in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);

            // std::cout<<" Beam energy "<<Ebeam_buff<<" Qval "<<Qval<<"\n";

            if (vertex_buff < 250 && vertex_buff > 150) {
               HKineRecoil->Fill(AngleRecoil_res, EnergyRecoil_res);
               if (EnergyRecoil < 6.0 && iEvent < xs_events[iFile])
                  HQval->Fill(ex_energy_exp);
            }

            Angle_sca_vs_Angle_rec->Fill(AngleRecoil, AngleSca);
            Vertex_vs_Angle_Rec->Fill(vertex, range_rec, AngleRecoil);

            for (Int_t i = 0; i < 10; i++) {

               if (vertex > 0.0 + i * 100 && vertex < 100.0 * (i + 1)) {
                  AngRange[i]->Fill(AngleRecoil, range_rec);
                  // std::cout<<radmean<<std::endl;
                  RadAng[i]->Fill(AngleRecoil, radmean);
                  // RadAng[i]->Fill(180-thetamean*180/TMath::Pi(),radmean);
               }
            }

         } else if (iEvent % 2 == 0)
            ICELoss->Fill(BeamEnergyLoss_IC);

         Range_vs_Energy_Sca->Fill(range_sca, EnergySca);
         Range_vs_Energy_Rec->Fill(range_rec, EnergyRecoil);
         Range_vs_Energy_Beam->Fill(range_beam, EnergyBeam);

      } // number of events

   } // file

   // c4->cd(1);
   // rad->Draw();

   c1->cd(1);
   Eloss_vs_Range_Sca->Draw("scat");
   c1->cd(2);
   Eloss_vs_Range_Rec->Draw("scat");
   c1->cd(3);
   ELossRatio->Draw();
   c1->cd(4);
   ICELoss->Draw();

   c2->cd(1);
   HKineRecoil->Draw("scat");
   c2->cd(2);
   HQval->Draw();
   /*tracks->Draw("col");
for(Int_t i=0;i<10;i++){
  c3->cd(i+1);
  AngRange[i]->Draw("col");
}
for(Int_t i=0;i<10;i++){
  c4->cd(i+1);
  RadAng[i]->Draw("col");
}
c5->cd(1);
//rad->Draw();
Angle_sca_vs_Angle_rec->Draw("zcol");
c7->cd();
Vertex_vs_Angle_Rec-> GetXaxis() -> SetTitle("Vertex Position [mm]");
Vertex_vs_Angle_Rec-> GetYaxis() -> SetTitle("Recoil Range [mm]");
Vertex_vs_Angle_Rec-> GetZaxis() -> SetTitle("Recoil Angle [deg]");
Vertex_vs_Angle_Rec->Draw("zcol");
c6->cd(1);
Range_vs_Energy_Sca->Draw();
c6->cd(2);
Range_vs_Energy_Rec->Draw();
c6->cd(3);
