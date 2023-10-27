Double_t omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

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

void C15_dd_ana()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   TH2F *bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 500, 0, 3);
   TH2F *bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 4000.0, 500, 0, 3);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_energy_lr = new TH2F("angle_vs_energy_lr", "angle_vs_energy_lr", 720, 0, 179, 500, 0, 100.0);
   TH2F *angle_vs_energy_t = new TH2F("angle_vs_energy_t", "angle_vs_energy_t", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);

   TH1F *HQval = new TH1F("HQval", "HQval", 600, -5, 55);
   TH2F *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);
   TH2F *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 200, -100, 100);

   TCutG *cutg = new TCutG("CUTG", 29);
   cutg->SetVarX("bro_vs_eloss");
   cutg->SetVarY("");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetPoint(0, 372.8158, 1.048105);
   cutg->SetPoint(1, 859.0976, 0.6636297);
   cutg->SetPoint(2, 1285.66, 0.5470116);
   cutg->SetPoint(3, 2394.724, 0.4486151);
   cutg->SetPoint(4, 3418.475, 0.4012391);
   cutg->SetPoint(5, 5338.008, 0.2937318);
   cutg->SetPoint(6, 6856.572, 0.2536443);
   cutg->SetPoint(7, 8972.325, 0.2445335);
   cutg->SetPoint(8, 10072.86, 0.2336006);
   cutg->SetPoint(9, 11284.3, 0.2244898);
   cutg->SetPoint(10, 12896.7, 0.2099125);
   cutg->SetPoint(11, 13928.99, 0.204446);
   cutg->SetPoint(12, 14654.14, 0.2099125);
   cutg->SetPoint(13, 14747.99, 0.1443148);
   cutg->SetPoint(14, 12461.61, 0.1169825);
   cutg->SetPoint(15, 10106.98, 0.1060495);
   cutg->SetPoint(16, 7564.667, 0.1024052);
   cutg->SetPoint(17, 5679.259, 0.1133382);
   cutg->SetPoint(18, 3589.1, 0.111516);
   cutg->SetPoint(19, 2343.537, 0.1534256);
   cutg->SetPoint(20, 1217.41, 0.1953353);
   cutg->SetPoint(21, 594.6285, 0.2226676);
   cutg->SetPoint(22, 253.3782, 0.3010204);
   cutg->SetPoint(23, 253.3782, 0.4941691);
   cutg->SetPoint(24, 219.2532, 0.6982507);
   cutg->SetPoint(25, 219.2532, 0.9879738);
   cutg->SetPoint(26, 270.4407, 1.069971);
   cutg->SetPoint(27, 372.8158, 1.049927);
   cutg->SetPoint(28, 372.8158, 1.048105);

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
   Double_t m_C15 = 15.0106 * 931.49401;
   Double_t m_C14 = 14.003242 * 931.49401;
   Double_t m_C13 = 13.00335484 * 931.49401;
   Double_t m_C12 = 12.00 * 931.49401;

   Double_t m_a = 4.00260325415 * 931.49401;
   Double_t m_O16 = 15.99491461956 * 931.49401;

   Double_t Ebeam_buff = 161.0;
   Double_t m_b;
   Double_t m_B;

   m_b = m_d;
   m_B = m_C15;

   TString FileName = "run_0014.root";
   // std::cout << " Opening File : " << FileName.Data() << std::endl;
   // TFile *file = new TFile(FileName.Data(), "READ");

   TString dir = "/home/yassid/fair_install/data/a1954/";

   std::vector<TString> files{"run_0014.root", "run_0015.root", "run_0016.root", "run_0017.root"};
   // files = {"run_0014NoCorr.root"};
   // files = {"run_0014.root"};

   for (auto iFile : files) {

      TFile *file = new TFile(iFile.Data(), "READ");

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");

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

               /*if (index != maxAIndex)
                  continue;*/
               auto track = patternTrackCand.at(index);

               Double_t theta = track.GetGeoTheta();
               Double_t rad = track.GetGeoRadius();

               Double_t B_f = 2.85;

               double bro = B_f * rad / TMath::Sin(theta) / 1000.0;
               double ener = 0;
               Double_t Am = 2.0;

               GetEnergy(Am, 1.0, bro, ener);

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
               std::size_t cnt = 0;
               Double_t zpos = 0;

               if (theta * TMath::RadToDeg() < 90) {
                  auto firstCluster = hitClusterArray->back();
                  zpos = firstCluster.GetPosition().Z();
                  auto it = hitClusterArray->rbegin();
                  while (it != hitClusterArray->rend()) {

                     if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.7)
                        break;
                     auto dir = (*it).GetPosition() - (*std::next(it, 1)).GetPosition();
                     eloss += (*it).GetCharge();
                     len += std::sqrt(dir.Mag2());
                     dedx += (*it).GetCharge();
                     // std::cout<<(*it).GetCharge()<<"\n";
                     it++;
                     ++cnt;
                  }
               } else if (theta * TMath::RadToDeg() > 90) {

                  auto firstCluster = hitClusterArray->front();
                  zpos = firstCluster.GetPosition().Z();
                  eloss += hitClusterArray->at(0).GetCharge();

                  cnt = 1;
                  for (auto iHitClus = 1; iHitClus < hitClusterArray->size(); ++iHitClus) {

                     if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.7)
                        break;
                     auto dir =
                        hitClusterArray->at(iHitClus).GetPosition() - hitClusterArray->at(iHitClus - 1).GetPosition();
                     len += std::sqrt(dir.Mag2());
                     eloss += hitClusterArray->at(iHitClus).GetCharge();
                     dedx += hitClusterArray->at(iHitClus).GetCharge();
                     // std::cout<<len<<" - "<<eloss<<" - "<<hitClusterArray->at(iHitClus).GetCharge()<<"\n";
                     ++cnt;
                  }
               }

               eloss /= cnt;
               dedx /= len;

               /*std::cout << " Brho : " << bro << " - Angle : " << theta * TMath::RadToDeg() << " - Radius : " << rad
                         << " - Energy :" << ener * Am << " - dE     :" << eloss << "\n";*/

               // Selection of events
               /*
                    if (zpos < 500.0 || zpos > 950)
                       continue;

                 if (theta * TMath::RadToDeg() < 60.0)
                       continue;*/

               // if (cutg->IsInside(eloss, bro)) { // Selection of protons

               angle_vs_energy->Fill(theta * TMath::RadToDeg(), ener * Am);
               auto [ex_energy_exp, theta_cm] = kine_2b(m_C15, m_d, m_b, m_B, Ebeam_buff, theta, ener * Am);

               HQval->Fill(ex_energy_exp);

               // Excitation energy vs Beam energy
               for (auto iEb = 0; iEb < 300; ++iEb) {
                  auto [Qdep, theta_cm_qdep] = kine_2b(m_C15, m_d, m_b, m_B, iEb, theta, ener * Am);
                  QvsEb->Fill(Qdep, iEb);
               }

               // Rough vertex
               QvsZpos->Fill(ex_energy_exp, zpos / 10.0);

               //} // protons

               bro_vs_eloss->Fill(eloss, bro);
               bro_vs_dedx->Fill(dedx, bro);

               angle_vs_energy_lr->Fill(theta * TMath::RadToDeg(), ener * Am);
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

   TString fileKine = "C14_pp_gs.txt";
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

   TString fileKine2 = "C14_pp_1m1.txt";
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

   TCanvas *c_kn_el_lr = new TCanvas();
   angle_vs_energy_lr->Draw("ZCOL");
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");

   TCanvas *c8 = new TCanvas();
   QvsEb->Draw("zcol");

   TCanvas *cQZ = new TCanvas();
   QvsZpos->Draw();

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
