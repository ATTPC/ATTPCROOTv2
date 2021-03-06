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


void plotFit(std::string fileFolder = "data_nobeamfit/")
{

   // Data histograms
   TH2F *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 179, 1000, 0, 100.0);
   TH1F *HQval = new TH1F("HQval", "HQval", 1000, -10, 10);
   TH1F *HIC = new TH1F("HIC", "HIC", 1000, 0, 4095);

   TH2F *Ang_Ener_Xtr = new TH2F("Ang_Ener_Xtr", "Ang_Ener_Xtr", 720, 0, 179, 1000, 0, 100.0);
   TH1F *HQval_Xtr = new TH1F("HQval_Xtr", "HQval_Xtr", 1000, -10, 10);
   
   TH2F *QvsAng = new TH2F("QvsAng", "QvsAng", 1000, -10, 10, 720, 0, 179);
   TH2F *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 10, 200, -100, 100);
   TH2F *ZposvsAng = new TH2F("ZposvsAng", "ZposvsAng", 200, -100, 100, 720, 0, 179);
   TH2F *QvsXpos = new TH2F("QvsXpos", "QvsXpos", 1000, -10, 10, 100, -10, 10);

   TH1F *hxpos_fit = new TH1F("hxpos_fit", "hxpos_fit", 100, -10, 10);
   TH1F *hypos_fit = new TH1F("hypos_fit", "hypos_fit", 100, -10, 10);
   TH1F *hzpos_fit = new TH1F("hzpos_fit", "hzpos_fit", 200, -100, 100);

   // PRA
   TH2F *Ang_Ener_PRA = new TH2F("Ang_Ener_PRA", "Ang_Ener_PRA", 720, 0, 179, 1000, 0, 100.0);

   // Correlations
   TH2F *Ang_AngPRA = new TH2F("Ang_AngPRA", "Ang_AngPRA", 720, 0, 179, 720, 0, 179);
   TH2F *Phi_PhiPRA = new TH2F("Phi_PhiPRA", "Phi_PhiPRA", 720, -179, 179, 720, -179, 179);
   TH2F *zfit_zPRA = new TH2F("zfit_zPRA", "zfit_zPRA", 1000, -100, 100, 1000, -100, 100);

   TH2F *Ang_Phi = new TH2F("Ang_Phi", "Ang_Phi", 720, 0, 179, 720, -179, 179);

   TH2F *x_Phi = new TH2F("x_Phi", "x_Phi", 1000, -10, 10, 720, -179, 179);
   TH2F *y_Phi = new TH2F("y_Phi", "y_Phi", 1000, -10, 10, 720, -179, 179);


   TH2F *x_y = new TH2F("x_y", "x_y", 1000, -10, 10, 1000, -10, 10);
   
   // Q-value calculation
   Double_t m_p = 1.007825 * 931.49401;
   Double_t m_d = 2.0135532 * 931.49401;
   Double_t m_Be10 = 10.013533818 * 931.49401;
   Double_t m_Be11 = 11.021657749 * 931.49401;
   Double_t m_beam = m_Be10;

   Double_t Ebeam_buff = 96.0; //(EnergyRecoil + EnergySca + ex_energy[iFile]);
   Double_t m_b;
   Double_t m_B;

      m_b = m_d;
      m_B = m_Be10;



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
         //files.push_back(fileFolder + line);
          files.push_back(line);
      }
   }

   // Plot data
   for (auto dataFile : files) {

      TFile rootfile(dataFile.c_str(), "READ");
      if (!rootfile.IsZombie()) {
         std::cout << " Opening file : " << dataFile << "\n";
         TTree *outputTree = (TTree *)rootfile.Get("outputTree");
         Float_t EFit, AFit, EPRA, APRA, Ex, PhiFit, PhiPRA, xiniPRA, yiniPRA, ziniPRA, xiniFit, yiniFit, ziniFit, IC;
         outputTree->SetBranchAddress("EFit", &EFit);
         outputTree->SetBranchAddress("AFit", &AFit);
         outputTree->SetBranchAddress("PhiFit", &PhiFit);
         outputTree->SetBranchAddress("EPRA", &EPRA);
         outputTree->SetBranchAddress("APRA", &APRA);
         outputTree->SetBranchAddress("PhiPRA", &PhiPRA);
         outputTree->SetBranchAddress("Ex", &Ex);
         outputTree->SetBranchAddress("xiniFit", &xiniFit);
         outputTree->SetBranchAddress("yiniFit", &yiniFit);
         outputTree->SetBranchAddress("ziniFit", &ziniFit);
         outputTree->SetBranchAddress("xiniPRA", &xiniPRA);
         outputTree->SetBranchAddress("yiniPRA", &yiniPRA);
         outputTree->SetBranchAddress("ziniPRA", &ziniPRA);
         outputTree->SetBranchAddress("IC", &IC);

	 std::vector<Float_t> *EFitVec = 0;
	 std::vector<Float_t> *AFitVec = 0;
	 std::vector<Float_t> *PhiFitVec = 0;
	 std::vector<Float_t> *EPRAVec = 0;
	 std::vector<Float_t> *APRAVec = 0;
	 std::vector<Float_t> *PhiPRAVec = 0;
	 std::vector<Float_t> *ExVec = 0;
	 std::vector<Float_t> *ExXtrVec = 0;
	 std::vector<Float_t> *xiniFitVec = 0;
	 std::vector<Float_t> *yiniFitVec = 0;
	 std::vector<Float_t> *ziniFitVec = 0;
	 std::vector<Float_t> *xiniPRAVec = 0;
	 std::vector<Float_t> *yiniPRAVec = 0;
	 std::vector<Float_t> *ziniPRAVec = 0;
	 std::vector<Float_t> *ICVec = 0;
	 std::vector<Float_t> *EFitXtrVec = 0;
	 std::vector<Float_t> *xiniFitXtrVec = 0;
	 std::vector<Float_t> *yiniFitXtrVec = 0;
	 std::vector<Float_t> *ziniFitXtrVec = 0;
	 std::vector<Float_t> *distXtrVec = 0;
	 std::vector<Float_t> *pValVec = 0;
	 std::vector<Float_t> *trackLengthVec = 0;
	 std::vector<Float_t> *POCAXtrVec = 0;
	 std::vector<Float_t> *trackIDVec = 0;

	 

	 outputTree->SetBranchAddress("EFitVec", &EFitVec);
	 outputTree->SetBranchAddress("AFitVec", &AFitVec);
	 outputTree->SetBranchAddress("PhiFitVec", &PhiFitVec);
	 outputTree->SetBranchAddress("EPRAVec", &EPRAVec);
	 outputTree->SetBranchAddress("APRAVec", &APRAVec);
	 outputTree->SetBranchAddress("PhiPRAVec", &PhiPRAVec);
	 outputTree->SetBranchAddress("ExVec", &ExVec);
	 outputTree->SetBranchAddress("ExXtrVec", &ExXtrVec);
	 outputTree->SetBranchAddress("xiniFitVec", &xiniFitVec);
	 outputTree->SetBranchAddress("yiniFitVec", &yiniFitVec);
	 outputTree->SetBranchAddress("ziniFitVec", &ziniFitVec);
	 outputTree->SetBranchAddress("xiniPRAVec", &xiniPRAVec);
	 outputTree->SetBranchAddress("yiniPRAVec", &yiniPRAVec);
	 outputTree->SetBranchAddress("ziniPRAVec", &ziniPRAVec);
	 outputTree->SetBranchAddress("ICVec", &ICVec);
	 outputTree->SetBranchAddress("EFitXtrVec", &EFitXtrVec);
	 outputTree->SetBranchAddress("xiniFitXtrVec", &xiniFitXtrVec);
	 outputTree->SetBranchAddress("yiniFitXtrVec", &yiniFitXtrVec);
	 outputTree->SetBranchAddress("ziniFitXtrVec", &ziniFitXtrVec);
	 outputTree->SetBranchAddress("distXtrVec", &distXtrVec);
	 outputTree->SetBranchAddress("pValVec", &pValVec);
	 outputTree->SetBranchAddress("trackLengthVec", &trackLengthVec);
	 outputTree->SetBranchAddress("POCAXtrVec", &POCAXtrVec);
	 outputTree->SetBranchAddress("trackIDVec", &trackIDVec);
	 
	 
	 

         Int_t nentries = (Int_t)outputTree->GetEntries();
         for (Int_t i = 0; i < nentries; i++) {
            outputTree->GetEntry(i);


	  if((IC>900  && IC<1300))
	  { 
	    //From std::vector
	    assert(EFitVec->size() == AFitVec->size() );

	    for(auto index = 0;index<EFitVec->size();++index)
	      {
		Ang_Ener->Fill((*AFitVec)[index], (*EFitVec)[index]);
		HQval->Fill((*ExVec)[index]);
		Ang_Ener_Xtr->Fill((*AFitVec)[index], (*EFitXtrVec)[index]);
		HQval_Xtr->Fill((*ExXtrVec)[index]);
	      }	

	    
            HIC->Fill(IC);
	    //if((IC>1800 && IC<2100)){
	    //if((IC>1000  && IC<1020) && AFit>50.0){
            // if((xiniFit<2.0 && xiniFit>-2.0) && (yiniFit<2.0 && yiniFit>-2.0))
            // {
            //Ang_Ener->Fill(AFit, EFit);
            Ang_Ener_PRA->Fill(APRA,EPRA);
	    
	    	    
            //HQval->Fill(Ex);
            hxpos_fit->Fill(xiniFit);
            hypos_fit->Fill(yiniFit);
            hzpos_fit->Fill(ziniFit);
            QvsAng->Fill(Ex, AFit);
            QvsZpos->Fill(Ex, ziniFit);
            ZposvsAng->Fill(ziniFit, AFit);
            Ang_AngPRA->Fill(AFit, APRA);
            zfit_zPRA->Fill(ziniFit, ziniPRA / 10.0);
            Phi_PhiPRA->Fill(PhiFit * TMath::RadToDeg(), PhiPRA);
            Ang_Phi->Fill(AFit, PhiFit * TMath::RadToDeg());
            x_Phi->Fill(xiniFit, PhiFit * TMath::RadToDeg());
            y_Phi->Fill(yiniFit, PhiFit * TMath::RadToDeg());
	    x_y->Fill(xiniFit,yiniFit);
	    QvsXpos->Fill(Ex,xiniFit);

	    // Excitation energy
            //Double_t ex_energy_exp = kine_2b(m_Be10, m_d, m_b, m_B, Ebeam_buff, AFit*TMath::DegToRad(),EFit);
	    //HQval->Fill(Ex);
	    }
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
   c1->Divide(2, 2);
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
   c1->cd(3);
   Ang_Ener_Xtr->SetMarkerStyle(20);
   Ang_Ener_Xtr->SetMarkerSize(0.5);
   Ang_Ener_Xtr->Draw();
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");
   c1->cd(4);
   HQval_Xtr->Draw();
   
   TCanvas *c2 = new TCanvas();
   c2->Divide(2, 3);
   c2->Draw();
   c2->cd(1);
   hxpos_fit->Draw();
   hypos_fit->Draw("SAMES");
   c2->cd(2);
   hzpos_fit->Draw();
   c2->cd(3);
   QvsAng->Draw();
   c2->cd(4);
   QvsZpos->Draw();
   c2->cd(5);
   ZposvsAng->Draw();
   c2->cd(6);
   QvsXpos->Draw();

   TCanvas *c3 = new TCanvas();
   c3->Divide(1, 2);
   c3->Draw();
   c3->cd(1);
   Ang_Ener_PRA->SetMarkerStyle(20);
   Ang_Ener_PRA->SetMarkerSize(0.5);
   Ang_Ener_PRA->Draw("COlZ");
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");

   TCanvas *c4 = new TCanvas();
   c4->Divide(2, 2);
   c4->Draw();
   c4->cd(1);
   Ang_AngPRA->Draw();
   c4->cd(2);
   zfit_zPRA->Draw();
   c4->cd(3);
   Phi_PhiPRA->Draw();
   c4->cd(4);
   Ang_Phi->Draw();

   TCanvas *c5 = new TCanvas();
   c5->Divide(2, 2);
   c5->Draw();
   c5->cd(1);
   x_Phi->Draw();
   c5->cd(2);
   y_Phi->Draw();
   c5->cd(3);
   HIC->Draw();
   c5->cd(4);
   x_y->Draw("zcol");
}
