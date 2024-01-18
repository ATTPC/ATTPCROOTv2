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

void pid_dev()
{

   TH2F *bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 1000, 0, 3);
   TH2F *bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 1000000, 1000, 0, 3);
   TH2F *broFit_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 1000, 0, 3);
   TH2F *broFit_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 1000000, 1000, 0, 3);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 500, 0, 80.0);

   std::vector<TString> files{"run_0011.root"};
   files.push_back("run_0013.root");
   files.push_back("run_0014.root");
   files.push_back("run_0015.root");
   files.push_back("run_0016.root");
   files.push_back("run_0017.root");
   files.push_back("run_0018.root");
   files.push_back("run_0019.root");
   files.push_back("run_0020.root");
   files.push_back("run_0021.root");
   files.push_back("run_0022.root");
   files.push_back("run_0023.root");
   files.push_back("run_0024.root");
   files.push_back("run_0025.root");
   files.push_back("run_0026.root");
   files.push_back("run_0027.root");
   files.push_back("run_0028.root");
   files.push_back("run_0029.root");
   files.push_back("run_0030.root");
   files.push_back("run_0031.root");
   files.push_back("run_0032.root");
   files.push_back("run_0033.root");
   files.push_back("run_0034.root");
   files.push_back("run_0035.root");
   files.push_back("run_0036.root");
   files.push_back("run_0037.root");
   files.push_back("run_0038.root");
   files.push_back("run_0039.root");
   files.push_back("run_0040.root");
   files.push_back("run_0041.root");
   files.push_back("run_0042.root");
   files.push_back("run_0043.root");
   files.push_back("run_0044.root");
   files.push_back("run_0045.root");
   files.push_back("run_0046.root");
   files.push_back("run_0047.root");
   files.push_back("run_0048.root");
   files.push_back("run_0049.root");
   files.push_back("run_0050.root");
   files.push_back("run_0051.root");
   files.push_back("run_0052.root");
   files.push_back("run_0053.root");
   files.push_back("run_0054.root");
   files.push_back("run_0055.root");
   files.push_back("run_0056.root");
   files.push_back("run_0057.root");
   files.push_back("run_0058.root");
   files.push_back("run_0059.root");
   files.push_back("run_0060.root");
   files.push_back("run_0061.root");
   files.push_back("run_0062.root");
   files.push_back("run_0063.root");
   files.push_back("run_0064.root");
   files.push_back("run_0065.root");
   files.push_back("run_0066.root");
   files.push_back("run_0067.root");
   files.push_back("run_0068.root");
   files.push_back("run_0069.root");
   files.push_back("run_0070.root");
   files.push_back("run_0071.root");
   files.push_back("run_0072.root");
   files.push_back("run_0073.root");
   files.push_back("run_0074.root");
   files.push_back("run_0075.root");
   files.push_back("run_0076.root");
   files.push_back("run_0077.root");
   files.push_back("run_0078.root");
   files.push_back("run_0079.root");
   files.push_back("run_0080.root");
   files.push_back("run_0081.root");
   files.push_back("run_0082.root");
   files.push_back("run_0083.root");
   files.push_back("run_0084.root");
   files.push_back("run_0085.root");
   files.push_back("run_0086.root");
   files.push_back("run_0087.root");
   files.push_back("run_0088.root");
   files.push_back("run_0089.root");
   files.push_back("run_0090.root");
   files.push_back("run_0091.root");
   files.push_back("run_0092.root");
   files.push_back("run_0093.root");
   files.push_back("run_0094.root");
   files.push_back("run_0095.root");
   files.push_back("run_0096.root");
   files.push_back("run_0097.root");
   files.push_back("run_0098.root");
   files.push_back("run_0099.root");
   files.push_back("run_0100.root");

   for (auto iFile : files) {

      TFile *file = new TFile(iFile.Data(), "READ");
      std::cout << " Processing file : " << iFile.Data() << "\n";

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader ReaderTracking("cbmsim", file);
      TTreeReaderValue<TClonesArray> patternArray(ReaderTracking, "AtPatternEvent");
      TTreeReaderValue<TClonesArray> eventHArray(ReaderTracking, "AtEventH");

      for (Int_t i = 0; i < nEvents; i++) {

         // eventArray->Clear();
         if (i % 1000 == 0)
            std::cout << " Event Number : " << i << "\n";

         ReaderTracking.Next();

         AtPatternEvent *patternEvent = (AtPatternEvent *)patternArray->At(0);

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

               auto track = patternTrackCand.at(index);

               Double_t theta = track.GetGeoTheta();
               Double_t rad = track.GetGeoRadius();

               if (theta * TMath::RadToDeg() > 90.0 || theta * TMath::RadToDeg() < 10.0)
                  continue;

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

                     if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.8)
                        break;
                     auto dir = (*it).GetPosition() - (*std::next(it, 1)).GetPosition();
                     eloss += (*it).GetCharge();
                     len = std::sqrt(dir.Mag2());
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

                     if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.8)
                        break;
                     auto dir =
                        hitClusterArray->at(iHitClus).GetPosition() - hitClusterArray->at(iHitClus - 1).GetPosition();
                     len = std::sqrt(dir.Mag2());
                     eloss += hitClusterArray->at(iHitClus).GetCharge();
                     dedx += hitClusterArray->at(iHitClus).GetCharge();
                     // std::cout<<len<<" - "<<eloss<<" - "<<hitClusterArray->at(iHitClus).GetCharge()<<"\n";
                     ++cnt;
                  }
               }

               eloss /= cnt;
               // dedx /= len;

               std::vector<ROOT::Math::XYZPoint> points;
               std::vector<double> charge;

               auto trackHits = &track.GetHitArray();
               std::size_t cntFit = 0;

               for (auto &hit : *trackHits) {

                  /*if (((Float_t)cntFit / (Float_t)trackHits->size()) > 0.2){
                  //std::cout<<" Processed hits "<<cntFit<<" over "<<trackHits->size()<<"\n";
                      break;
                  }*/

                  auto position = hit->GetPosition();
                  ROOT::Math::XYZPoint point(position.X(), position.Y(), position.Z());
                  points.push_back(point);

                  ++cntFit;
               }

               // Brho from fit with less points
               auto patternCircle = std::make_unique<AtPatterns::AtPatternCircle2D>();
               patternCircle->AtPattern::FitPattern(points);
               std::vector<double> par = patternCircle->GetPatternPar();

               // std::cout<<" Radius from fit : "<<par.at(2)<<" - Radius from pattern : "<<rad<<"\n";
               double broFit = B_f * par.at(2) / TMath::Sin(theta) / 1000.0;

               bro_vs_eloss->Fill(eloss, bro);
               bro_vs_dedx->Fill(dedx, bro);
               broFit_vs_eloss->Fill(eloss, broFit);
               broFit_vs_dedx->Fill(dedx, broFit);

               // angle_vs_energy_lr->Fill(theta * TMath::RadToDeg(), ener * Am);
            }
         }
      }

      file->Close();
   }

   TCanvas *c_PID = new TCanvas();
   c_PID->Divide(1, 2);
   c_PID->cd(1);
   bro_vs_eloss->Draw("colz");
   c_PID->cd(2);
   bro_vs_dedx->Draw("colz");

   TCanvas *c_PIDFit = new TCanvas();
   c_PIDFit->Divide(1, 2);
   c_PIDFit->cd(1);
   broFit_vs_eloss->Draw("colz");
   c_PIDFit->cd(2);
   broFit_vs_dedx->Draw("colz");
}