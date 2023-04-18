
void GetEnergy(Double_t M, Double_t IZ, Double_t BRO, Double_t &E);

void C14_pp_ana()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   TH2F *bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 500, 0, 3, 500, 0, 25000.0);
   TH2F *bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 500, 0, 3, 500, 0, 5000.0);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_energy_lr = new TH2F("angle_vs_energy_lr", "angle_vs_energy_lr", 720, 0, 179, 500, 0, 100.0);
   TH2F *angle_vs_energy_t = new TH2F("angle_vs_energy_t", "angle_vs_energy_t", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);

   TFile *f_d_cut = new TFile("./d_cut.root");
   TCutG *mycut_d = (TCutG *)f_d_cut->Get("d_mycut");
   f_d_cut->Close();

   TFile *f_p_cut = new TFile("./p_cut.root");
   TCutG *mycut_p = (TCutG *)f_p_cut->Get("p_mycut");
   f_p_cut->Close();

   TCutG *cutg = new TCutG("CUTG", 19);
   cutg->SetVarX("bro_vs_eloss");
   cutg->SetVarY("");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetPoint(0, 0.1628849, 13670.29);
   cutg->SetPoint(1, 0.1537682, 7091.346);
   cutg->SetPoint(2, 0.2753241, 3106.022);
   cutg->SetPoint(3, 0.4789303, 1840.84);
   cutg->SetPoint(4, 0.8101702, 1113.36);
   cutg->SetPoint(5, 0.9438817, 670.5464);
   cutg->SetPoint(6, 1.132293, 702.1759);
   cutg->SetPoint(7, 1.186994, 986.8419);
   cutg->SetPoint(8, 1.205227, 1587.803);
   cutg->SetPoint(9, 1.089749, 2220.395);
   cutg->SetPoint(10, 0.7706645, 3137.652);
   cutg->SetPoint(11, 0.5822528, 4339.575);
   cutg->SetPoint(12, 0.4728525, 6743.421);
   cutg->SetPoint(13, 0.3847245, 9400.304);
   cutg->SetPoint(14, 0.3543355, 11614.37);
   cutg->SetPoint(15, 0.3148298, 13670.29);
   cutg->SetPoint(16, 0.2662074, 14492.66);
   cutg->SetPoint(17, 0.159846, 13860.07);
   cutg->SetPoint(18, 0.1628849, 13670.29);

   TString FileName = "run_0060.root";
   // std::cout << " Opening File : " << FileName.Data() << std::endl;
   // TFile *file = new TFile(FileName.Data(), "READ");

   TString dir = "/home/yassid/fair_install/data/a1954/";

   std::vector<TString> files{"run_0055.root", "run_0056.root", "run_0057.root", "run_0058.root", "run_0059.root",
                              "run_0060.root", "run_0061.root", "run_0062.root", "run_0064.root", "run_0065.root",
                              "run_0066.root", "run_0068.root", "run_0069.root"};

   for (auto iFile : files) {

      TFile *file = new TFile(iFile.Data(), "READ");

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");

      for (Int_t i = 0; i < nEvents; i++) {

         // eventArray->Clear();

         std::cout << " Event Number : " << i << "\n";

         Reader1.Next();

         AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);

         if (patternEvent) {
            std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
            std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";
            for (auto track : patternTrackCand) {

               Double_t theta = track.GetGeoTheta();
               Double_t rad = track.GetGeoRadius();

               Double_t B_f = 2.85;

               double bro = B_f * rad / TMath::Sin(theta) / 1000.0;
               double ener = 0;
               Double_t Am = 1.0;

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

               std::cout << " Brho : " << bro << " - Angle : " << theta * TMath::RadToDeg() << " - Radius : " << rad
                         << " - Energy :" << ener * Am << " - dE     :" << eloss << "\n";

               if (mycut_p->IsInside(bro, dedx))
                  angle_vs_energy->Fill(theta * TMath::RadToDeg(), ener * Am);
               // if ( mycut_d -> IsInside(bro, dedx) ) angle_vs_energy_t->Fill(theta * TMath::RadToDeg(), ener * Am);
               if (cutg->IsInside(bro, eloss))
                  angle_vs_energy_t->Fill(theta * TMath::RadToDeg(), ener / 2.0);

               bro_vs_eloss->Fill(bro, eloss);
               bro_vs_dedx->Fill(bro, dedx);

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

   TCanvas *c_kn_el = new TCanvas();
   c_kn_el->Divide(3, 1);
   c_kn_el->cd(1);
   angle_vs_energy->Draw("colz");
   Kine_AngRec_EnerRec->SetLineColor(kRed);
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");

   c_kn_el->cd(2);
   bro_vs_dedx->Draw("colz");
   mycut_p->Draw("SAME");
   mycut_d->Draw("SAME");
   // TCanvas *c_kn_t = new TCanvas();
   // c_kn_t->cd();
   c_kn_el->cd(3);
   angle_vs_energy_t->Draw("colz");

   TCanvas *c_PID_eloss = new TCanvas();
   TCanvas *c_PID_dedx = new TCanvas();
   c_PID_eloss->cd();
   bro_vs_eloss->Draw("colz");
   c_PID_dedx->cd();
   bro_vs_dedx->Draw("colz");
   mycut_p->Draw("SAME");
   mycut_d->Draw("SAME");

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
