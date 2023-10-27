TCutG *cutp;
TCutG *cutd;
TCutG *cutt;
TCutG *cuthe;
TCutG *cutd2;
TCutG *cutp2;

TCutG *cutHe3;
TCutG *cutlow;
TCutG *cutmid;
TCutG *cuthigh;
TCutG *cutpvv;

TCutG *cut0vv;
TCutG *cut1vv;
TCutG *cut2vv;

TH2F *bro_vs_eloss;
TH2F *bro_vs_eloss_mid;
TH2F *bro_vs_eloss_high;
TH2F *bro_vs_eloss_uncut;
TH2F *bro_vs_dedx;
TH2F *angle_vs_energy;
TH2F *angle_vs_energy_lr;
TH2F *angle_vs_energy_p;
TH2F *angle_vs_momentum;

TH1F *HQval;
TH1F *HQvalp;
TH2F *QvsEb;
TH2F *QvsZpos;
TH2F *vx_vs_vy;
TH1F *track_len;

TH1F *henergyIC;

void draw_kinematics(std::string filename);

void create_cuts()
{
   TFile fileHe("cut/He3.root", "read");
   cutHe3 = new TCutG(*dynamic_cast<TCutG *>(fileHe.Get("CUTG")));

   // Proton cut
   {
      TFile filep("cut/p_eloss.root", "read");
      cutp = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
   }
   {
      TFile filep("cut/d_eloss.root", "read");
      cutd = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
   }
   {
      TFile filep("cut/he3_eloss.root", "read");
      cuthe = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
   }
   {
      TFile filep("cut/t_eloss.root", "read");
      cutt = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
   }

   {
      TFile filep("cut/p_dedx.root", "read");
      cutp2 = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
   }
   {
      TFile filep("cut/d_dedx.root", "read");
      cutd2 = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
   }
   {

      {
         TFile filep("cut/dedxLow.root", "read");
         cutlow = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
      }

      {
         TFile filep("cut/dedxMid.root", "read");
         cutmid = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
      }

      {
         TFile filep("cut/dedxTop.root", "read");
         cuthigh = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
      }
      {
         TFile filep("cut/p_vv.root", "read");
         cutpvv = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
      }
      {
         TFile filep("cut/el_vv.root", "read");
         cut0vv = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
      }
      {
         TFile filep("cut/1st_vv.root", "read");
         cut1vv = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
      }
      {
         TFile filep("cut/2nd_vv.root", "read");
         cut2vv = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));
      }
   }
}

void create_histograms()
{

   bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 1000, 0, 3);
   bro_vs_eloss_mid = new TH2F("bro_vs_eloss_mid", "bro_vs_eloss mid", 4000, 0, 25000.0, 1000, 0, 3);
   bro_vs_eloss_high = new TH2F("bro_vs_eloss_high", "bro_vs_eloss high", 4000, 0, 25000.0, 1000, 0, 3);
   bro_vs_eloss_uncut = new TH2F("bro_vs_eloss_uncut", "bro_vs_eloss_uncut", 4000, 0, 25000.0, 1000, 0, 3);
   bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 4000.0, 1000, 0, 3);
   angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 500, 0, 80.0);
   angle_vs_energy_lr = new TH2F("angle_vs_energy_lr", "angle_vs_energy_lr", 720, 0, 179, 500, 0, 100.0);
   angle_vs_energy_p = new TH2F("angle_vs_energy_t", "angle_vs_energy_t", 720, 0, 179, 500, 0, 80.0);
   angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);

   vx_vs_vy = new TH2F("vx_vs_vy", "vx_vs_vy", 1000, 0, 10, 1000, 0, 10);

   HQval = new TH1F("HQval", "HQval", 600, -5, 55);
   HQvalp = new TH1F("HQvalp", "HQvalp", 600, -5, 55);
   QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);
   QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 200, -100, 100);
   track_len = new TH1F("track_len", "Track Length", 100, 0, 1000);

   henergyIC = new TH1F("henergyIC", "henergyIC", 2048, 0, 2047);
}

void C15_dd_ana_merged()
{
   TChain *chain = new TChain("a2091");
   int numFiles = chain->Add("mergedData/run_*_merged.root");
   std::cout << "Loaded " << numFiles << std::endl;

   TTreeReader reader(chain);
   TTreeReaderValue<int> eventID(reader, "eventID");
   TTreeReaderValue<int> trackID(reader, "trackID");
   TTreeReaderValue<float> theta(reader, "theta");
   TTreeReaderValue<float> bro(reader, "brho");
   TTreeReaderValue<float> ener(reader, "energy");
   TTreeReaderValue<float> zpos(reader, "length");
   TTreeReaderValue<float> len(reader, "zpos");
   TTreeReaderValue<float> eloss(reader, "eloss");
   TTreeReaderValue<float> dedx(reader, "dedx");
   TTreeReaderValue<float> Qval(reader, "Qval");
   TTreeReaderValue<std::vector<float>> energyIC(reader, "energyIC");

   create_cuts();
   create_histograms();

   while (reader.Next()) {

      /*
       std::cout << "id " <<*eventID << std::endl;
       std::cout << *trackID << std::endl;if (true) {
       std::cout << "theta " << *theta << std::endl;
       std::cout << *bro <<{TFile filep("cut/p_vv.root","read");
   cutpvv = new TCutG(*dynamic_cast<TCutG *>(filep.Get("CUTG")));} std::endl;
       std::cout << "en " << *ener << std::endl;
       std::cout << *zpos << std::endl;
       std::cout << *len << std::endl;
       std::cout << *eloss << std::endl;
       std::cout << *dedx << std::endl;
       std::cout << *Qval << std::endl;
       std::cout << *energyIC << std::endl;
       */
      bool goodEn = true;
      for (auto en : *energyIC) {
         // Fill energy hiscontinuetorgram
         if (en > 900 && en < 1200) {

            henergyIC->Fill(en);
         } else
            goodEn = false;
         break;
      }

      if (!goodEn)
         continue;
      if ((*theta) * TMath::RadToDeg() > 90)
         continue;

      float Am = 2;
      *ener = (*ener) * 3 / 2.;
      Double_t vx = TMath::Sin(*theta) * TMath::Sqrt((*ener) * Am);
      Double_t vy = TMath::Cos(*theta) * TMath::Sqrt((*ener) * Am);

      // if(vx < 0.45 || vy < 0.05)
      // continue;

      if (cutp->IsInside(*eloss, *bro) && cutp2->IsInside(*dedx, *bro)) {
         angle_vs_energy_p->Fill((*theta) * TMath::RadToDeg(), (*ener) * Am);
         HQvalp->Fill(*Qval);
      }

      track_len->Fill(*len);
      bro_vs_eloss_uncut->Fill(*eloss, *bro);

      // if (*dedx < 100)
      // continue;
      // if (cutd->IsInside(*eloss, *bro) && ( true || cutd2->IsInside(*dedx, *bro))) //d
      // if (cutd->IsInside(*eloss, *bro) && cutd2->IsInside(*dedx, *bro))
      if (cutt->IsInside(*eloss, *bro)) {
         vx_vs_vy->Fill(vx, vy);
         // if (true || (cut0vv->IsInside(vx, vy) || cut1vv->IsInside(vx, vy) || cut2vv->IsInside(vx, vy))) {
         bro_vs_eloss->Fill(*eloss, *bro);
         bro_vs_dedx->Fill(*dedx, *bro);
         angle_vs_energy->Fill((*theta) * TMath::RadToDeg(), (*ener) * Am);
         HQval->Fill(*Qval);
         //}
      }

      // angle_vs_energy->Fill((*theta) * TMath::RadToDeg(), (*ener) * Am);
      //  if (cutd->IsInside(*eloss, *bro) && cutd2->IsInside(*dedx, *bro)) { // Selection of d
      if (cutt->IsInside(*eloss, *bro)) {

         angle_vs_energy_lr->Fill((*theta) * TMath::RadToDeg(), (*ener) * Am);
      } // cut
      // break;
   }

   TCanvas *cvxvy = new TCanvas();
   vx_vs_vy->Draw("zcol");

   TCanvas *c_kn_el_lr = new TCanvas();
   angle_vs_energy_lr->Draw("ZCOL");
   draw_kinematics("C15_dhe3_gs.txt");
   draw_kinematics("C15_dt_gs.txt");
   continue draw_kinematics("C15_da_gs.txt");

   /*
      TCanvas *c8 = new TCanvas();
      QvsEb->Draw("zcol");

      TCanvas *cQZ = new TCanvas();
      QvsZpos->Draw();
      */

   TCanvas *c_kn_el = new TCanvas();
   c_kn_el->Divide(2, 1);
   c_kn_el->cd(1);
   angle_vs_energy->Draw("colz");

   c_kn_el->cd(2);
   angle_vs_energy_p->Draw("colz");

   TCanvas *c_PID_eloss = new TCanvas();
   TCanvas *c_PID_eloss_uncut = new TCanvas();
   TCanvas *c_PID_dedx = new TCanvas();
   c_PID_eloss->cd();
   bro_vs_eloss->Draw("colz");
   c_PID_eloss_uncut->cd();
   bro_vs_eloss_uncut->Draw("colz");
   // cutg->Draw("l");
   // cutp->Draw("l");
   c_PID_dedx->cd();
   bro_vs_dedx->Draw("colz");

   angle_vs_energy->GetXaxis()->SetTitle("#theta (deg)");
   angle_vs_energy->GetYaxis()->SetTitle("E (MeV)");
   angle_vs_energy->SetTitle("gate on d");

   angle_vs_energy_p->GetXaxis()->SetTitle("#theta (deg)");
   angle_vs_energy_p->GetYaxis()->SetTitle("E (MeV)");
   angle_vs_energy_p->SetTitle("gate on p");

   bro_vs_dedx->GetXaxis()->SetTitle("Brho ");
   bro_vs_dedx->GetYaxis()->SetTitle("dE/dx (au)");
   bro_vs_dedx->SetTitle("PID (p/d)");

   angle_vs_energy->SetStats(0);
   angle_vs_energy_p->SetStats(0);
   bro_vs_dedx->SetStats(0);

   TCanvas *c_ExEner = new TCanvas();
   c_ExEner->Divide(2, 1);
   c_ExEner->cd(1);
   HQval->Draw();
   c_ExEner->cd(2);
   HQvalp->Draw();

   TCanvas *c_IC = new TCanvas();
   henergyIC->Draw();

   TCanvas *c_len = new TCanvas();
   track_len->Draw();
}

void draw_kinematics(std::string fileName)
{
   std::array<double, 20000> thetaCms;
   std::array<double, 20000> thetaLabRec;
   std::array<double, 20000> thetaLabSca;
   std::array<double, 20000> enerLabRec;
   std::array<double, 20000> enerLabSca;
   std::array<double, 20000> momLabRec;

   std::ifstream *kineStr = new std::ifstream(fileName);
   Int_t numKin = 0;

   if (!kineStr->fail()) {
      while (!kineStr->eof()) {
         *kineStr >> thetaCms[numKin] >> thetaLabRec[numKin] >> enerLabRec[numKin] >> thetaLabSca[numKin] >>
            enerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics (" << fileName << ") file found for this reaction!" << std::endl;

   (new TGraph(numKin, thetaLabRec.data(), enerLabRec.data()))->Draw("SAME");
}