void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, Double_t &E);

void C15_ana()
{

   FairRunAna *run = new FairRunAna();

   // Histograms
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 1000, 0, 100.0);
   TH2F *bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 1000, 0, 3);
   TH2F *bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 4000.0, 1000, 0, 3);
   TH2F *angle_vs_bro = new TH2F("angle_vs_bro", "angle_vs_bro", 720, 0, 179, 1000, 0, 3);

   std::vector<TString> files = {"./data/C15_d3He_gs.root", "./data/C15_d4He_gs_1.root", "./data/C15_d4He_gs_2.root",
                                 "./data/C15_dt_gs.root", "./data/C15_dd_gs.root"};

   for (auto filename : files) {

      std::cout << " Opening File : " << filename.Data() << std::endl;
      TFile *file = new TFile(filename.Data(), "READ");

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      for (Int_t i = 0; i < nEvents; i++) {

         std::cout << " Event Number : " << i << "\n";

         Reader1.Next();

         AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);

         if (patternEvent) {
            std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
            std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";
            for (auto track : patternTrackCand) {

               Double_t theta = track.GetGeoTheta();
               Double_t rad = track.GetGeoRadius();
               Double_t phi = track.GetGeoPhi();

               Double_t B_f = 2.85;

               double bro = B_f * rad / TMath::Sin(theta) / 1000.0;
               double ener = 0;
               Double_t Am = 3.0;

               GetEnergy(Am, 2.0, bro, ener);

               angle_vs_energy->Fill(180.0 - theta * TMath::RadToDeg(), ener * Am);
               angle_vs_bro->Fill(180.0 - theta * TMath::RadToDeg(), bro);

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
               } else if (theta * TMath::RadToDeg() > 90) {

                  auto firstCluster = hitClusterArray->front();
                  zpos = firstCluster.GetPosition().Z();
                  eloss += hitClusterArray->at(0).GetCharge();

                  cnt = 1;
                  for (auto iHitClus = 1; iHitClus < hitClusterArray->size(); ++iHitClus) {

                     if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.5)
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

               std::cout << " Brho : " << bro << " - Theta : " << theta * TMath::RadToDeg()
                         << " - Phi : " << phi * TMath::RadToDeg() << " - Radius : " << rad
                         << " - Energy :" << ener * Am << "\n";

               bro_vs_eloss->Fill(eloss, bro);
               bro_vs_dedx->Fill(dedx, bro);

            } // if pattern
         }    // patternevent

      } // Events

   } // files

   TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->Draw();
   c1->cd(1);
   angle_vs_energy->Draw();
   c1->cd(2);
   angle_vs_bro->Draw();
   c1->cd(3);
   bro_vs_eloss->Draw("colz");
   c1->cd(4);
   bro_vs_dedx->Draw("colz");
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