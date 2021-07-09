void plotFit(std::string fileFolder = "dd_520/")
{

   // Data histograms
   TH2F *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 179, 1000, 0, 100.0);
   TH1F *HQval = new TH1F("HQval", "HQval", 1000, -10, 10);

   TH1F *hxpos_fit = new TH1F("hxpos_fit", "hxpos_fit", 100, -10, 10);
   TH1F *hypos_fit = new TH1F("hypos_fit", "hypos_fit", 100, -10, 10);
   TH1F *hzpos_fit = new TH1F("hzpos_fit", "hzpos_fit", 200, -100, 100);

   // Find every valid file
   // std::system("find ./dd_520 -maxdepth 1 -printf \"%f\n\" >test.txt"); // execute the UNIX command "ls -l >test.txt"
   std::system("find ./ -maxdepth 1 -printf \"%f\n\" >test.txt"); // execute the UNIX command "ls -l >test.txt"
   std::ifstream file;
   file.open("test.txt");
   std::string line;
   std::string fileType = "fit_analysis";
   std::string fileExt = "*.root";
   std::vector<std::string> files;

   while (std::getline(file, line)) {
      std::istringstream iss(line);
      if (line.find(fileType) != std::string::npos) {
         std::cout << " Found fit file : " << line << "\n";
         // files.push_back(fileFolder+line);
         files.push_back(line);
      }
   }

   // Plot data
   for (auto dataFile : files) {

      TFile rootfile(dataFile.c_str(), "READ");
      if (!rootfile.IsZombie()) {
         std::cout << " Opening file : " << dataFile << "\n";
         TTree *outputTree = (TTree *)rootfile.Get("outputTree");
         Float_t EFit, AFit, EPRA, APRA, Ex, xiniPRA, yiniPRA, ziniPRA, xiniFit, yiniFit, ziniFit;
         outputTree->SetBranchAddress("EFit", &EFit);
         outputTree->SetBranchAddress("AFit", &AFit);
         outputTree->SetBranchAddress("EPRA", &EPRA);
         outputTree->SetBranchAddress("APRA", &APRA);
         outputTree->SetBranchAddress("Ex", &Ex);
         outputTree->SetBranchAddress("xiniFit", &xiniFit);
         outputTree->SetBranchAddress("yiniFit", &yiniFit);
         outputTree->SetBranchAddress("ziniFit", &ziniFit);

         Int_t nentries = (Int_t)outputTree->GetEntries();
         for (Int_t i = 0; i < nentries; i++) {
            outputTree->GetEntry(i);
            Ang_Ener->Fill(AFit, EFit);
            if (ziniFit > 20 && ziniFit < 30 && EFit > 2.5)
               HQval->Fill(Ex);
            hxpos_fit->Fill(xiniFit);
            hypos_fit->Fill(yiniFit);
            hzpos_fit->Fill(ziniFit);
         }
      }
   }

   // Adding kinematic lines
   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "Be10pp_el.txt";
   std::ifstream *kineStr = new std::ifstream(fileKine.Data());
   Int_t numKin = 0;

   if (!kineStr->fail()) {
      while (!kineStr->eof()) {
         *kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   fileKine = "Be10pp_in_2+1.txt";
   std::ifstream *kineStr2 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr2->fail()) {
      while (!kineStr2->eof()) {
         *kineStr2 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr2->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec_in = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   fileKine = "Be10dp_gs.txt";
   std::ifstream *kineStr3 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr3->fail()) {
      while (!kineStr3->eof()) {
         *kineStr3 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr3->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec_dp = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   fileKine = "Be10dp_first.txt";
   std::ifstream *kineStr4 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr4->fail()) {
      while (!kineStr4->eof()) {
         *kineStr4 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr4->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec_dp_first = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   TCanvas *c1 = new TCanvas();
   c1->Divide(1, 2);
   c1->Draw();
   c1->cd(1);
   Ang_Ener->SetMarkerStyle(20);
   Ang_Ener->SetMarkerSize(0.5);
   Ang_Ener->Draw();
   Kine_AngRec_EnerRec->SetLineWidth(1);
   Kine_AngRec_EnerRec->SetLineColor(kRed);
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_AngRec_EnerRec_in->SetLineWidth(1);
   Kine_AngRec_EnerRec_in->SetLineColor(kBlue);
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp->SetLineWidth(1);
   Kine_AngRec_EnerRec_dp->SetLineColor(kGreen);
   Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp_first->SetLineWidth(1);
   Kine_AngRec_EnerRec_dp_first->SetLineColor(kViolet);
   Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");
   c1->cd(2);
   HQval->Draw();

   TCanvas *c2 = new TCanvas();
   c2->Divide(1, 2);
   c2->Draw();
   c2->cd(1);
   hxpos_fit->Draw();
   hypos_fit->Draw("SAMES");
   c2->cd(2);
   hzpos_fit->Draw();
}
