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

void plotFit(std::string fileFolder = "data/")
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

   TH2F *QvsAng_Xtr = new TH2F("QvsAng_Xtr", "QvsAng_Xtr", 1000, -10, 10, 720, 0, 179);

   TH1F *hxpos_fit = new TH1F("hxpos_fit", "hxpos_fit", 100, -10, 10);
   TH1F *hypos_fit = new TH1F("hypos_fit", "hypos_fit", 100, -10, 10);
   TH1F *hzpos_fit = new TH1F("hzpos_fit", "hzpos_fit", 200, -100, 100);

   TH1F *hxpos_fit_Xtr = new TH1F("hxpos_fit_Xtr", "hxpos_fit_Xtr", 100, -10, 10);
   TH1F *hypos_fit_Xtr = new TH1F("hypos_fit_Xtr", "hypos_fit_Xtr", 100, -10, 10);
   TH1F *hzpos_fit_Xtr = new TH1F("hzpos_fit_Xtr", "hzpos_fit_Xtr", 200, -100, 100);

   // PRA
   TH2F *Ang_Ener_PRA = new TH2F("Ang_Ener_PRA", "Ang_Ener_PRA", 720, 0, 179, 1000, 0, 100.0);

   // Correlations
   TH2F *Ang_AngPRA = new TH2F("Ang_AngPRA", "Ang_AngPRA", 720, 0, 179, 720, 0, 179);
   TH2F *Phi_PhiPRA = new TH2F("Phi_PhiPRA", "Phi_PhiPRA", 720, -179, 179, 720, -179, 179);
   TH2F *zfit_zPRA = new TH2F("zfit_zPRA", "zfit_zPRA", 1000, -100, 100, 1000, -100, 100);

   TH2F *Ang_Phi = new TH2F("Ang_Phi", "Ang_Phi", 720, 0, 179, 720, -179, 179);

   TH2F *x_Phi = new TH2F("x_Phi", "x_Phi", 1000, -10, 10, 720, -179, 179);
   TH2F *y_Phi = new TH2F("y_Phi", "y_Phi", 1000, -10, 10, 720, -179, 179);

   TH2F *x_y_Xtr = new TH2F("x_y_Xtr", "x_y_Xtr", 1000, -10, 10, 1000, -10, 10);

   TH1F *fChi2H = new TH1F("fChi2H", "fChi2H", 100, -10, 10);
   TH1F *bChi2H = new TH1F("bChi2H", "bChi2H", 100, -10, 10);
   TH1F *fNdfH = new TH1F("fNdfH", "fNdfH", 100, -10, 10);
   TH1F *bNdfH = new TH1F("bNdfH", "fbNdfH", 100, -10, 10);

   TH1I *ICMultH = new TH1I("ICMultH", "ICMultH", 100, 0, 100);
   TH1I *ICTimeH = new TH1I("ICTimeH", "ICTimeH", 512, 0, 511);
   TH1I *ICQH = new TH1I("ICQH", "ICQH", 1000, 0, 20000);
   TH1I *ICAH = new TH1I("ICAH", "ICAH", 1000, 0, 4096);

   TH2F *ICEvsTime = new TH2F("ICEvsTime", "ICEvsTime", 1000, 0, 4095, 512, 0, 511);

   TH2F *ICCorr = new TH2F("ICCorr", "ICCorr", 1000, 0, 4096, 1000, 0, 4096);

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
         // files.push_back(fileFolder + line);
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
         Int_t ICMult;
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
         outputTree->SetBranchAddress("ICMult", &ICMult);

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
         std::vector<Int_t> *ICTimeVec = 0;
         std::vector<Float_t> *EFitXtrVec = 0;
         std::vector<Float_t> *xiniFitXtrVec = 0;
         std::vector<Float_t> *yiniFitXtrVec = 0;
         std::vector<Float_t> *ziniFitXtrVec = 0;
         std::vector<Float_t> *distXtrVec = 0;
         std::vector<Float_t> *pValVec = 0;
         std::vector<Float_t> *trackLengthVec = 0;
         std::vector<Float_t> *POCAXtrVec = 0;
         std::vector<Float_t> *trackIDVec = 0;
         std::vector<Float_t> *fChi2Vec = 0;
         std::vector<Float_t> *bChi2Vec = 0;
         std::vector<Float_t> *fNdfVec = 0;
         std::vector<Float_t> *bNdfVec = 0;
         std::vector<Float_t> *ICEVec = 0;

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
         outputTree->SetBranchAddress("ICTimeVec", &ICTimeVec);
         outputTree->SetBranchAddress("EFitXtrVec", &EFitXtrVec);
         outputTree->SetBranchAddress("xiniFitXtrVec", &xiniFitXtrVec);
         outputTree->SetBranchAddress("yiniFitXtrVec", &yiniFitXtrVec);
         outputTree->SetBranchAddress("ziniFitXtrVec", &ziniFitXtrVec);
         outputTree->SetBranchAddress("distXtrVec", &distXtrVec);
         outputTree->SetBranchAddress("pValVec", &pValVec);
         outputTree->SetBranchAddress("trackLengthVec", &trackLengthVec);
         outputTree->SetBranchAddress("POCAXtrVec", &POCAXtrVec);
         outputTree->SetBranchAddress("trackIDVec", &trackIDVec);
         outputTree->SetBranchAddress("fChi2Vec", &fChi2Vec);
         outputTree->SetBranchAddress("bChi2Vec", &bChi2Vec);
         outputTree->SetBranchAddress("fNdfVec", &fNdfVec);
         outputTree->SetBranchAddress("bNdfVec", &bNdfVec);
         outputTree->SetBranchAddress("ICEVec", &ICEVec);

         Int_t nentries = (Int_t)outputTree->GetEntries();
         for (Int_t i = 0; i < nentries; i++) {
            outputTree->GetEntry(i);

            //  if (ICMult != 1)
            //	continue;

            Int_t ICIndex = 0;
            Int_t iQindex = 0;
            Float_t ICQ = 0;
            Float_t ICBL = 0;
            Float_t ICA = 0;

            try {

               if (ICEVec->size() > 0) {

                  for (iQindex = 0; iQindex < 20; ++iQindex)
                     ICBL += (*ICEVec)[iQindex];

                  ICBL /= 20.0;

                  for (iQindex = 60; iQindex < 80; ++iQindex) {
                     ICQ += (*ICEVec)[iQindex] - ICBL;

                     Float_t max = (*ICEVec)[iQindex] - ICBL;

                     if (max > ICA)
                        ICA = max;
                  }

                  ICQH->Fill(ICQ);
                  ICAH->Fill(ICA);
               }
               //}

               for (ICIndex = 0; ICIndex < ICVec->size(); ++ICIndex) {

                  HIC->Fill((*ICVec)[ICIndex]);
                  ICEvsTime->Fill((*ICVec)[ICIndex], (*ICTimeVec)[ICIndex]);
                  ICTimeH->Fill((*ICTimeVec)[ICIndex]);
                  ICCorr->Fill((*ICVec)[ICIndex], ICA);
               }

               //    if (ICA < 500 || ICA > 900)
               //   continue;

               // if(ICMult==1){

            } catch (...) {
            }

            // if(ICA>850)
            // continue;

            //	   	     if ((IC >0 && IC < 1620)) {
            // From std::vector
            assert(EFitVec->size() == AFitVec->size());

            for (auto index = 0; index < EFitVec->size(); ++index) {

               Double_t angle = (*AFitVec)[index];

               if (dataFile.find("sim") != std::string::npos) {
                  angle = (*AFitVec)[index];
               }

               fChi2H->Fill((*fChi2Vec)[index]);
               bChi2H->Fill((*bChi2Vec)[index]);
               fNdfH->Fill((*fNdfVec)[index]);
               bNdfH->Fill((*bNdfVec)[index]);

               // if((*AFitVec)[index]>20.0 && (*EFitVec)[index]>4.0){
               //	if( ((*xiniFitXtrVec)[index]<0.3 && (*xiniFitXtrVec)[index]>-0.3) && ((*yiniFitXtrVec)[index]<0.3 &&
               //(*yiniFitXtrVec)[index]>-0.3) ){
               Ang_Ener->Fill(angle, (*EFitVec)[index]);
               HQval->Fill((*ExVec)[index]);
               Ang_Ener_Xtr->Fill((angle), (*EFitXtrVec)[index]);
               HQval_Xtr->Fill((*ExXtrVec)[index]);
               hxpos_fit_Xtr->Fill((*xiniFitXtrVec)[index]);
               hypos_fit_Xtr->Fill((*yiniFitXtrVec)[index]);
               hzpos_fit_Xtr->Fill((*ziniFitXtrVec)[index]);
               x_y_Xtr->Fill((*xiniFitXtrVec)[index], (*yiniFitXtrVec)[index]);
               QvsAng_Xtr->Fill((*ExXtrVec)[index], AFit);
               // }// x-y
               //}//Energy and angle

               //	  	 	  	}//IC
               ICMultH->Fill(ICMult);

               Ang_Ener_PRA->Fill(APRA, EPRA);

               // HQval->Fill(Ex);
               // hxpos_fit->Fill(xiniFit);
               // hypos_fit->Fill(yiniFit);
               // hzpos_fit->Fill(ziniFit);

               QvsAng->Fill(Ex, AFit);
               QvsZpos->Fill(Ex, ziniFit);
               ZposvsAng->Fill(ziniFit, AFit);
               Ang_AngPRA->Fill(AFit, APRA);
               zfit_zPRA->Fill(ziniFit, ziniPRA / 10.0);
               Phi_PhiPRA->Fill(PhiFit * TMath::RadToDeg(), PhiPRA);
               Ang_Phi->Fill(AFit, PhiFit * TMath::RadToDeg());
               x_Phi->Fill(xiniFit, PhiFit * TMath::RadToDeg());
               y_Phi->Fill(yiniFit, PhiFit * TMath::RadToDeg());

               QvsXpos->Fill(Ex, xiniFit);

               // Excitation energy
               // Double_t ex_energy_exp = kine_2b(m_Be10, m_d, m_b, m_B, Ebeam_buff, AFit*TMath::DegToRad(),EFit);
               // HQval->Fill(Ex);
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

   fileKine = "Be10pp_el_9AMeV.txt";
   std::ifstream *kineStr5 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr5->fail()) {
      while (!kineStr5->eof()) {
         *kineStr5 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec_9AMeV = new TGraph(numKin, ThetaLabRec, EnerLabRec);

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
   Kine_AngRec_EnerRec_9AMeV->SetLineWidth(1);
   Kine_AngRec_EnerRec_9AMeV->SetLineColor(kGreen + 10);
   Kine_AngRec_EnerRec_9AMeV->Draw("SAME");

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
   Kine_AngRec_EnerRec_9AMeV->Draw("SAME");
   c1->cd(4);
   HQval_Xtr->Draw();

   TCanvas *c2 = new TCanvas();
   c2->Divide(2, 3);
   c2->Draw();
   c2->cd(1);
   hxpos_fit_Xtr->Draw();
   hypos_fit_Xtr->Draw("SAMES");
   c2->cd(2);
   hzpos_fit_Xtr->Draw();
   c2->cd(3);
   QvsAng_Xtr->Draw();
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
   c5->Divide(3, 3);
   c5->Draw();
   c5->cd(1);
   x_Phi->Draw();
   c5->cd(2);
   y_Phi->Draw();
   c5->cd(3);
   // HIC->Draw();
   ICCorr->Draw("zcol");
   c5->cd(4);
   x_y_Xtr->Draw("zcol");
   c5->cd(5);
   ICMultH->Draw();
   c5->cd(6);
   ICTimeH->Draw();
   c5->cd(7);
   ICEvsTime->Draw("zcol");
   c5->cd(8);
   ICQH->Draw();
   c5->cd(9);
   ICAH->Draw();

   TCanvas *cChi = new TCanvas();
   cChi->Divide(2, 2);
   cChi->Draw();
   cChi->cd(1);
   fChi2H->Draw();
   cChi->cd(2);
   bChi2H->Draw();
   cChi->cd(3);
   fNdfH->Draw();
   cChi->cd(4);
   bNdfH->Draw();
}
