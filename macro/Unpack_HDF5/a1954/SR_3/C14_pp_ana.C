#include <TMath.h>
#include <TGraph2D.h>
#include <TRandom2.h>
#include <TStyle.h>
#include <TCanvas.h>
#include <TF2.h>
#include <TH1.h>
#include <Math/Functor.h>
#include <TPolyLine3D.h>
#include <Math/Vector3D.h>
#include <Fit/Fitter.h>
 
#include <cassert>
 
using namespace ROOT::Math;

Double_t omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

void line(double t, const double *p, double &x, double &y, double &z) {
   // a parametric line is define from 6 parameters but 4 are independent
   // x0,y0,z0,z1,y1,z1 which are the coordinates of two points on the line
   // can choose z0 = 0 if line not parallel to x-y plane and z1 = 1;
   x = p[0] + p[1]*t;
   y = p[2] + p[3]*t;
   z = t;
}

 
bool first = true;
struct SumDistance2 {
   // the TGraph is a data member of the object
   TGraph2D *fGraph;
 
   SumDistance2(TGraph2D *g) : fGraph(g) {}
 
   // calculate distance line-point
   double distance2(double x,double y,double z, const double *p) {
      // distance line point is D= | (xp-x0) cross  ux |
      // where ux is direction of line and x0 is a point in the line (like t = 0)
      XYZVector xp(x,y,z);
      XYZVector x0(p[0], p[2], 0. );
      XYZVector x1(p[0] + p[1], p[2] + p[3], 1. );
      XYZVector u = (x1-x0).Unit();
      double d2 = ((xp-x0).Cross(u)).Mag2();
      return d2;
   }
 
   // implementation of the function to be minimized
   double operator() (const double *par) {
      assert(fGraph != nullptr);
      double * x = fGraph->GetX();
      double * y = fGraph->GetY();
      double * z = fGraph->GetZ();
      int npoints = fGraph->GetN();
      double sum = 0;
      for (int i  = 0; i < npoints; ++i) {
         double d = distance2(x[i],y[i],z[i],par);
         sum += d;
      }
      if (first) {
         std::cout << "Total Initial distance square = " << sum << std::endl;
      }
      first = false;
      return sum;
   }
 
};
 
std::tuple<double, double>
kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject)
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

   theta_cm = theta_cm * TMath::RadToDeg();
   return std::make_tuple(Ex, theta_cm);
}

void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, Double_t &E);

void C14_pp_ana()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   TH2F *bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 500, 0, 3);
   TH2F *bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 4000.0, 500, 0, 3);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_energy_lr = new TH2F("angle_vs_energy_lr", "angle_vs_energy_lr", 720, 0, 179, 500, 0, 100.0);
   TH2F *angle_vs_energy_t = new TH2F("angle_vs_energy_t", "angle_vs_energy_t", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);
   TH2F *angle_vs_angle = new TH2F("angle_vs_angle", "angle_vs_angle", 720, 0, 179, 720, 0, 179);
   TH1F *vertex_distribution = new TH1F("vertex_distribution", "Vertex", 100, 0, 1000);

   TH1F *HQval = new TH1F("HQval", "HQval", 600, -5, 55);
   TH2F *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);
   TH2F *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 200, -100, 100);

   TCutG *cutg = new TCutG("CUTG", 29);
   cutg->SetVarX("bro_vs_eloss");
   cutg->SetVarY("");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetPoint(0, 198.1692, 0.954291);
   cutg->SetPoint(1, 691.6648, 0.5708955);
   cutg->SetPoint(2, 1115.763, 0.4533582);
   cutg->SetPoint(3, 2218.417, 0.3554104);
   cutg->SetPoint(4, 3251.673, 0.3078358);
   cutg->SetPoint(5, 5171.68, 0.2014925);
   cutg->SetPoint(6, 6675.299, 0.1623134);
   cutg->SetPoint(7, 8803.499, 0.1511194);
   cutg->SetPoint(8, 9906.153, 0.1427239);
   cutg->SetPoint(9, 11109.05, 0.1315298);
   cutg->SetPoint(10, 12720.62, 0.1175373);
   cutg->SetPoint(11, 13746.17, 0.1119403);
   cutg->SetPoint(12, 14478.7, 0.1175373);
   cutg->SetPoint(13, 14571.23, 0.05037311);
   cutg->SetPoint(14, 12288.81, 0.02518655);
   cutg->SetPoint(15, 9929.286, 0.01399252);
   cutg->SetPoint(16, 7384.699, 0.0083955);
   cutg->SetPoint(17, 5503.247, 0.02238804);
   cutg->SetPoint(18, 3421.312, 0.01958953);
   cutg->SetPoint(19, 2164.441, 0.06156714);
   cutg->SetPoint(20, 1046.365, 0.1035448);
   cutg->SetPoint(21, 421.7844, 0.1287313);
   cutg->SetPoint(22, 90.21706, 0.2070895);
   cutg->SetPoint(23, 74.79532, 0.4029851);
   cutg->SetPoint(24, 51.66271, 0.6072761);
   cutg->SetPoint(25, 51.66271, 0.8955224);
   cutg->SetPoint(26, 97.92792, 0.9766791);
   cutg->SetPoint(27, 205.8801, 0.9570895);
   cutg->SetPoint(28, 198.1692, 0.954291);

   // NB: Not used
   // Q-value calculation
   Double_t m_p = 1.007825 * 931.49401;
   Double_t m_d = 2.0135532 * 931.49401;
   Double_t m_t = 3.016049281 * 931.49401;
   Double_t m_He3 = 3.016029 * 931.49401;
   Double_t m_Be10 = 10.013533818 * 931.49401;
   Double_t m_Be11 = 11.021657749 * 931.49401;
   Double_t m_Li9 = 9.026790 * 931.49401;
   Double_t m_beam = m_Be10;
   Float_t aMass = 4.00260325415;
   Float_t O16Mass = 15.99491461956;
   Double_t m_C14 = 14.003242 * 931.49401;
   Double_t m_C13 = 13.00335484 * 931.49401;
   Double_t m_C12 = 12.00 * 931.49401;

   Double_t m_a = 4.00260325415 * 931.49401;
   Double_t m_O16 = 15.99491461956 * 931.49401;

   Double_t Ebeam_buff = 18.1;
   Double_t m_b;
   Double_t m_B;

   m_b = m_a;
   m_B = m_Be10;

   //TString FileName = "run_0062.root";
   // std::cout << " Opening File : " << FileName.Data() << std::endl;
   // TFile *file = new TFile(FileName.Data(), "READ");

   TString dir = "/home/david/PhD/PhD-14-02/attpcroot/ATTPCROOTv2/macro/Unpack_HDF5/a1954/";

   std::vector<TString> files{"run_0062_1.25.root"};
//"run_0062.root","run_0063.root","run_0064.root","run_0065.root","run_0066.root","run_0067.root","run_0070.root","run_0071.root","run_0072.root",
   for (auto iFile : files) {

      TFile *file = new TFile(iFile.Data(), "READ");

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
Int_t nfail = 0;

      for (Int_t i = 0; i < nEvents; i++) {

         // eventArray->Clear();
         if (i % 1000 == 0)
            std::cout << " Event Number : " << i << "\n";

         Reader1.Next();

         AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);

         if (patternEvent) {
            std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
            // std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";

            // Find track with largets angle
            auto itMax =
               std::max_element(patternTrackCand.begin(), patternTrackCand.end(),
                                [](const auto &a, const auto &b) { return b.GetGeoTheta() > a.GetGeoTheta(); });
            Int_t maxAIndex = std::distance(patternTrackCand.begin(), itMax);

            // for (auto track : patternTrackCand) {
            for (auto index = 0; index < patternTrackCand.size(); ++index) {

               if (index != maxAIndex)
                  continue;

               if (patternTrackCand.size() > 1){

                  auto track1 = patternTrackCand.at(0);
                  auto track2 = patternTrackCand.at(1);

                  Double_t theta1 = track1.GetGeoTheta();
                  Double_t theta2 = track2.GetGeoTheta();     

                  auto hitClusterArray1 = track1.GetHitClusterArray();
                  auto firstCluster1 = hitClusterArray1->back();
                  auto zpos1 = firstCluster1.GetPosition().Z();

                  auto hitClusterArray2 = track2.GetHitClusterArray();
                  auto firstCluster2 = hitClusterArray2->back();
                  auto zpos2 = firstCluster2.GetPosition().Z();               
               
                  angle_vs_angle->Fill(theta1 * TMath::RadToDeg(), theta2 * TMath::RadToDeg());


                  vector<double> p1_line1;
                  vector<double> p2_line1;
                  vector<double> p1_line2;
                  vector<double> p2_line2;

               
                  auto onep = hitClusterArray1->at(hitClusterArray1->size() - 1);
                  auto twop = hitClusterArray1->at(hitClusterArray1->size() - 2);

                  auto onep2 = hitClusterArray2->at(hitClusterArray2->size() - 1);
                  auto twop2 = hitClusterArray2->at(hitClusterArray2->size() - 2);

                  p1_line1 = {onep.GetPosition().X(), onep.GetPosition().Y(), onep.GetPosition().Z()};
                  p2_line1 = {twop.GetPosition().X(), twop.GetPosition().Y(), twop.GetPosition().Z()};

                  p1_line2 = {onep2.GetPosition().X(), onep2.GetPosition().Y(), onep2.GetPosition().Z()};
                  p2_line2 = {twop2.GetPosition().X(), twop2.GetPosition().Y(), twop2.GetPosition().Z()};
                  
                  vector<double> dirvec1 = {p2_line1[0] - p1_line1[0], p2_line1[1] - p1_line1[1], p2_line1[2] - p1_line1[2]};
                  vector<double> dirvec2 = {p2_line2[0] - p1_line2[0], p2_line2[1] - p1_line2[1], p2_line2[2] - p1_line2[2]};

                  double a = dirvec1[0], b = -dirvec2[0], c = p1_line2[0] - p1_line1[0];
                  double d = dirvec1[1], e = -dirvec2[1], f = p1_line2[1] - p1_line1[1];
                  double g = dirvec1[2], h = -dirvec2[2], i = p1_line2[2] - p1_line1[2];

    
                  double denominator = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);


                  double t = (c * (e * i - f * h) - b * (f * i - c * h) + a * (f * h - e * i)) / denominator;
                  double s = (a * (f * i - c * h) - c * (d * i - f * g) + b * (d * h - e * g)) / denominator;

    
                  vector<double> vertex = {p1_line1[0] + t * dirvec1[0], p1_line1[1] + t * dirvec1[1], p1_line1[2] + t * dirvec1[2]};
    
                  vertex_distribution->Fill(vertex[2]);
                     
               }



               auto track = patternTrackCand.at(index);

               Double_t theta = track.GetGeoTheta();
               Double_t rad = track.GetGeoRadius();

               Double_t B_f = 2.0;

               double bro = B_f * rad / TMath::Sin(theta) / 1000.0;
               double ener = 0;
               Double_t Am = 4.0;

               GetEnergy(Am, 2.0, bro, ener);
               angle_vs_energy_lr->Fill(theta * TMath::RadToDeg(), ener * Am);
               // if (track.GetHitArray()->size() > 80)
               // angle_vs_energy->Fill(theta * TMath::RadToDeg(), ener * Am);

               // std::cout << " Brho : " << bro << " - Angle : " << theta * TMath::RadToDeg() << " - Radius : " << rad
               //          << " - Energy :" << ener * Am << "\n";

               /*std::vector<AtHit> *hitArray = track.GetHitArray();
                     for (auto hit : *hitArray) {
                        TVector3 pos = hit.GetPosition();
                        int TB = hit.GetTimeStamp();
                        std::cout << " Pos : " << pos.X() << "   " << pos.Y() << "       " << pos.Z() << " " << TB <<
                  "\n";
                   }*/

               // PID
               Double_t len = 0;
               Double_t eloss = 0;
               Double_t dedx = 0;

               // Energy loss from ADC
               auto hitClusterArray = track.GetHitClusterArray();
               auto firstCluster = hitClusterArray->back();
               auto zpos = firstCluster.GetPosition().Z();
               std::size_t cnt = 0;

               auto it = hitClusterArray->rbegin();
               while (it != hitClusterArray->rend()) {

                  if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.5)
                     break;
                  auto dir = (*it).GetPosition() - (*std::next(it, 1)).GetPosition();
                  eloss += (*it).GetCharge();
                  len += std::sqrt(dir.Mag2());
                  dedx += (*it).GetCharge();
                  // std::cout<<(*it).GetCharge()<<"\n";
                  it++;
                  ++cnt;
               }
               eloss /= cnt;
               dedx /= len;

               /*std::cout << " Brho : " << bro << " - Angle : " << theta * TMath::RadToDeg() << " - Radius : " << rad
                         << " - Energy :" << ener * Am << " - dE     :" << eloss << "\n";*/

               // Selection of events
               if (zpos < 500.0 || zpos > 950)
                  continue;

               if (theta * TMath::RadToDeg() < 13.0)
                  continue;

               if (cutg->IsInside(eloss, bro)) { // Selection of protons

                 
                  auto [ex_energy_exp, theta_cm] = kine_2b(m_C14, m_p, m_b, m_B, Ebeam_buff, theta, ener);

                  HQval->Fill(ex_energy_exp);

                  // Excitation energy vs Beam energy
                  for (auto iEb = 0; iEb < 300; ++iEb) {
                     auto [Qdep, theta_cm_qdep] = kine_2b(m_C14, m_p, m_b, m_B, iEb, theta, ener);
                     QvsEb->Fill(Qdep, iEb);
                  }

                  // Rough vertex
                  QvsZpos->Fill(ex_energy_exp, zpos / 10.0);

               } // protons

               bro_vs_eloss->Fill(eloss, bro);
               bro_vs_dedx->Fill(dedx, bro);

               
            }
         }

      } // nEvents

      file->Close();

   } // Files

   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "10Be_a_gs.txt";
   std::ifstream *kineStr = new std::ifstream(fileKine.Data());
   Int_t numKin = 0;

   if (!kineStr->fail()) {
      while (!kineStr->eof()) {
         *kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         // numKin++;

         // MomLabRec[numKin] =( pow(EnerLabRec[numKin] + M_Ener,2) - TMath::Power(M_Ener, 2))/1000.0;
         // std::cout<<" Momentum : " <<MomLabRec[numKin]<<"\n";
         // Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec = new TGraph(numKin, ThetaLabRec, EnerLabRec);
   TGraph *Kine_AngRec_AngSca = new TGraph(numKin, ThetaLabRec, ThetaLabSca);

   TString fileKine2 = "10Be_a_1s.txt";
   std::ifstream *kineStr2 = new std::ifstream(fileKine2.Data());
   numKin = 0;

   if (!kineStr2->fail()) {
      while (!kineStr2->eof()) {
         *kineStr2 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr2->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_1m1 = new TGraph(numKin, ThetaLabRec, EnerLabRec);
   TGraph *Kine_1m1_ang_ang = new TGraph(numKin, ThetaLabRec, ThetaLabSca);
   
   TCanvas *c_kn_el_lr = new TCanvas();
   angle_vs_energy_lr->Draw("ZCOL");
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");

   TCanvas *c8 = new TCanvas();
   QvsEb->Draw("zcol");

   TCanvas *cQZ = new TCanvas();
   QvsZpos->Draw();

   TCanvas *angle_angle = new TCanvas();
   angle_vs_angle->Draw("colz");
   Kine_AngRec_AngSca->SetLineColor(kRed);
   Kine_AngRec_AngSca->Draw("SAME");
   Kine_1m1_ang_ang->SetLineColor(kBlack);
   Kine_1m1_ang_ang->Draw("SAME");

   TCanvas *c_vertex = new TCanvas();
   vertex_distribution->Draw();

   TCanvas *c_kn_el = new TCanvas();
   c_kn_el->Divide(3, 1);
   c_kn_el->cd(1);
   angle_vs_energy->Draw("colz");
   Kine_AngRec_EnerRec->SetLineColor(kRed);
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");
   c_kn_el->cd(2);
   bro_vs_dedx->Draw("colz");
   c_kn_el->cd(3);
   angle_vs_energy_t->Draw("colz");

   TCanvas *c_PID_eloss = new TCanvas();
   TCanvas *c_PID_dedx = new TCanvas();
   c_PID_eloss->cd();
   bro_vs_eloss->Draw("colz");
   cutg->Draw("l");
   c_PID_dedx->cd();
   bro_vs_dedx->Draw("colz");

   angle_vs_energy->GetXaxis()->SetTitle("#theta (deg)");
   angle_vs_energy->GetYaxis()->SetTitle("E (MeV)");
   angle_vs_energy->SetTitle("gate on p");

   angle_vs_energy_t->GetXaxis()->SetTitle("#theta (deg)");
   angle_vs_energy_t->GetYaxis()->SetTitle("E (MeV)");
   angle_vs_energy_t->SetTitle("gate on d");

   bro_vs_dedx->GetXaxis()->SetTitle("Brho ");
   bro_vs_dedx->GetYaxis()->SetTitle("dE/dx (au)");
   bro_vs_dedx->SetTitle("PID (p/d)");

   angle_vs_energy->SetStats(0);
   angle_vs_energy_t->SetStats(0);
   bro_vs_dedx->SetStats(0);

   TCanvas *c_ExEner = new TCanvas();
   HQval->Draw();
}

void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, Double_t &E)
{

   // Energy per nucleon
   Float_t AM = 931.5;
   Float_t X = BRO / 0.1439 * IZ / M;
   X = pow(X, 2);
   X = 2. * AM * X;
   X = X + pow(AM, 2);
   E = TMath::Sqrt(X) - AM;
}
