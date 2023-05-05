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

void plotFit_full(std::string fileFolder = "data/")
{

   // Data histograms
   TH2F *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 179, 1000, 0, 100.0);
   TH1F *HQval = new TH1F("HQval", "HQval", 1000, -5, 15);
   TH1F *HIC = new TH1F("HIC", "HIC", 1000, 0, 4095);

   TH2F *Ang_Ener_Xtr = new TH2F("Ang_Ener_Xtr", "Ang_Ener_Xtr", 720, 0, 179, 1000, 0, 100.0);
   TH1F *HQval_Xtr = new TH1F("HQval_Xtr", "HQval_Xtr", 600, -5, 55);
   TH1F *HQCorr = new TH1F("HQCorr", "HQCorr", 600, -5, 55);
   TH1F *HQCorrArray[10];

   TH1F *HQval_Xtr_recalc = new TH1F("HQval_Xtr_recalc", "HQval_Xtr_recalc", 1200, -5, 55);
   TH1F *HQval_Xtr_recalc_cutgs = new TH1F("HQval_Xtr_recalc_cutgs", "HQval_Xtr_recalc_cutgs", 1200, -5, 55);

   TH2F *QvsAng = new TH2F("QvsAng", "QvsAng", 1000, -10, 10, 720, 0, 179);
   TH2F *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 10, 200, -100, 100);
   TH2F *QcorrvsZpos = new TH2F("QcorrvsZpos", "QcorrvsZpos", 1000, -10, 10, 200, -100, 100);
   TH2F *ZposvsAng = new TH2F("ZposvsAng", "ZposvsAng", 200, -100, 100, 720, 0, 179);
   TH2F *QvsXpos = new TH2F("QvsXpos", "QvsXpos", 1000, -10, 10, 100, -10, 10);

   TH2F *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -10, 10, 150, 0, 300);

   TH2F *QvsTrackLengthH = new TH2F("QvsTrackLengthH", "QvsTrackLengthH", 1000, -10, 10, 1000, 0, 1000);

   TH2F *QvsAng_Xtr = new TH2F("QvsAng_Xtr", "QvsAng_Xtr", 1000, -10, 10, 720, 0, 179);

   TH1F *hxpos_fit = new TH1F("hxpos_fit", "hxpos_fit", 100, -10, 10);
   TH1F *hypos_fit = new TH1F("hypos_fit", "hypos_fit", 100, -10, 10);
   TH1F *hzpos_fit = new TH1F("hzpos_fit", "hzpos_fit", 200, -100, 100);

   TH1F *hxpos_fit_Xtr = new TH1F("hxpos_fit_Xtr", "hxpos_fit_Xtr", 100, -10, 10);
   TH1F *hypos_fit_Xtr = new TH1F("hypos_fit_Xtr", "hypos_fit_Xtr", 100, -10, 10);
   TH1F *hzpos_fit_Xtr = new TH1F("hzpos_fit_Xtr", "hzpos_fit_Xtr", 300, -100, 200);

   // PRA
   TH2F *Ang_Ener_PRA = new TH2F("Ang_Ener_PRA", "Ang_Ener_PRA", 720, 0, 179, 1000, 0, 100.0);
   TH1F *PhiPRAH = new TH1F("PhiPRAH", "PhiPRAH", 720, -179, 179);

   // Correlations
   TH2F *Ang_AngPRA = new TH2F("Ang_AngPRA", "Ang_AngPRA", 720, 0, 179, 720, 0, 179);
   TH2F *Phi_PhiPRA = new TH2F("Phi_PhiPRA", "Phi_PhiPRA", 720, -179, 179, 720, -179, 179);
   TH2F *zfit_zPRA = new TH2F("zfit_zPRA", "zfit_zPRA", 1000, -100, 100, 1000, -100, 100);

   TH2F *Ang_Phi = new TH2F("Ang_Phi", "Ang_Phi", 720, 0, 179, 720, -179, 179);

   TH2F *x_Phi = new TH2F("x_Phi", "x_Phi", 1000, -10, 10, 720, -179, 179);
   TH2F *y_Phi = new TH2F("y_Phi", "y_Phi", 1000, -10, 10, 720, -179, 179);

   TH2F *x_y_Fit = new TH2F("x_y_Fit", "x_y_Fit", 1000, -10, 10, 1000, -10, 10);

   TH2F *x_y_Xtr = new TH2F("x_y_Xtr", "x_y_Xtr", 1000, -10, 10, 1000, -10, 10);

   TH1F *POCAXtrH = new TH1F("POCAXtrH", "POCAXtrH", 1000, -10, 10);

   TH1F *tracklengthH = new TH1F("tracklengthH", "tracklengthH", 1000, 0, 1000);

   TH2F *ZposvsEvH = new TH2F("ZposvsEvH", "ZposvsEvH", 200, -100, 100, 1000, 0, 10000);

   TH2F *ZposvsRad = new TH2F("ZposvsRad", "ZposvsRad", 200, -100, 100, 1000, 0, 10);

   TH1F *fChi2H = new TH1F("fChi2H", "fChi2H", 100, -10, 10);
   TH1F *bChi2H = new TH1F("bChi2H", "bChi2H", 100, -10, 10);
   TH1F *fNdfH = new TH1F("fNdfH", "fNdfH", 100, -10, 10);
   TH1F *bNdfH = new TH1F("bNdfH", "fbNdfH", 100, -10, 10);

   TH1F *fChi2NH = new TH1F("fChi2NH", "fChi2NH", 1000, 0, 10);

   TH2F *QvsChi2 = new TH2F("QvsChi2", "QvsChi2", 1000, -10, 10, 1000, 0, 0.1);

   TH1I *ICMultH = new TH1I("ICMultH", "ICMultH", 100, 0, 100);
   TH1I *ICTimeH = new TH1I("ICTimeH", "ICTimeH", 512, 0, 511);
   TH1I *ICQH = new TH1I("ICQH", "ICQH", 1000, 0, 20000);
   TH1I *ICAH = new TH1I("ICAH", "ICAH", 1000, 0, 4096);

   TH1I *particleQH = new TH1I("particleQH", "particleQH", 21, -10.5, 10.5);

   TH1I *eventMultH = new TH1I("eventMultH", "eventMultH", 10, 0, 10);

   TH2F *ICEvsTime = new TH2F("ICEvsTime", "ICEvsTime", 1000, 0, 4095, 512, 0, 511);

   TH1F *ExZ[10];

   TH2F *ELossvsBrho = new TH2F("ELossvsBrho", "ELossvsBrho", 2000, 0, 2000, 1000, 0, 10);

   for (auto iHist = 0; iHist < 10; ++iHist)
      ExZ[iHist] = new TH1F(Form("ExZ[%i]", iHist), Form("ExZ[%i]", iHist), 1000, -2, 18);

   for (auto iHist = 0; iHist < 10; ++iHist)
      HQCorrArray[iHist] = new TH1F(Form("HQCorrArray[%i]", iHist), Form("HQCorrArray[%i]", iHist), 600, -5, 55);

   TH2F *QvsEvent = new TH2F("QvsEvent", "QvsEvent", 1000, -10, 10, 1000, 0, 1000);

   TH2F *fOrbZvsfOrbLength = new TH2F("fOrbZvsfOrbLength", "fOrbZvsfOrbLength", 500, -250, 250, 500, 0, 500);
   TH2F *fOrbZvsEFit = new TH2F("fOrbZvsEFit", "fOrbZvsEFit", 1000, 0, 200, 1000, 0, 100);
   TH2F *fOrbLengthvsEFit = new TH2F("fOrbLengthvsEFit", "fOrbLengthvsEFit", 500, 0, 500, 1000, 0, 100);
   TH1F *PhiOrbZH = new TH1F("PhiOrbZH", "PhiOrbZH", 1000, 0, 100);
   TH2F *fOrbZvsEx = new TH2F("fOrbZvsEx", "fOrbZvsEx", 200, 0, 200, 100, -10, 10);
   TH2F *fOrbZvsZ = new TH2F("fOrbZvsZ", "fOrbZvsZ", 1000, -200, 200, 1000, -200, 200);
   TH1F *HQCorrOrbZ = new TH1F("HQCorrOrbZ", "HQCorrOrbZ", 600, -5, 55);
   TH2F *fOrbZvsAFit = new TH2F("fOrbZvsAFit", "fOrbZvsAFit", 1000, 0, 200, 720, 0, 180);
   TH2F *fOrbZvsMomLoss = new TH2F("fOrbZvsMomLoss", "fOrbZvsMomLoss", 1000, 0, 200, 100, 0, 10);
   TH2F *fOrbLengthvsMomLoss = new TH2F("fOrbLengthvsMomLoss", "fOrbLengthvsMomLoss", 500, 0, 500, 100, 0, 10);

   // Cut for gs 75-76 mm vertex
   TCutG *cutGS = new TCutG("CUTGS", 8);
   cutGS->SetVarX("Ang_Ener_Xtr");
   cutGS->SetVarY("");
   cutGS->SetTitle("Graph");
   cutGS->SetFillStyle(1000);
   cutGS->SetPoint(0, 42.47056, 58.17587);
   cutGS->SetPoint(1, 70.22306, 15.78246);
   cutGS->SetPoint(2, 68.79986, 7.909399);
   cutGS->SetPoint(3, 61.56523, 13.05717);
   cutGS->SetPoint(4, 47.80758, 33.04264);
   cutGS->SetPoint(5, 35.94754, 53.93653);
   cutGS->SetPoint(6, 38.43815, 57.57025);
   cutGS->SetPoint(7, 42.47056, 58.17587);

   // NB: Not used
   // Q-value calculation
   Double_t m_p = 1.007825 * 931.49401;
   Double_t m_d = 2.0135532 * 931.49401;
   Double_t m_He3 = 3.016029 * 931.49401;
   Double_t m_Be10 = 10.013533818 * 931.49401;
   Double_t m_Be11 = 11.021657749 * 931.49401;
   Double_t m_Li9 = 9.026790 * 931.49401;
   Double_t m_beam = m_Be10;
   Float_t aMass = 4.00260325415;
   Float_t O16Mass = 15.99491461956;

   Double_t m_a = 4.00260325415 * 931.49401;
   Double_t m_O16 = 15.99491461956 * 931.49401;

   Double_t Ebeam_buff = 90.5; //(EnergyRecoil + EnergySca + ex_energy[iFile]);
   Double_t m_b;
   Double_t m_B;

   m_b = m_p;
   m_B = m_Be11;

   // Find every valid file
   std::system("find ./data -maxdepth 1 -printf \"%f\n\" >test.txt"); // execute the UNIX command "ls -l
   // >test.txt"
   // std::system("find ./ -maxdepth 1 -printf \"%f\n\" >test.txt"); // execute the UNIX command "ls -l >test.txt"
   std::ifstream file;
   file.open("test.txt");
   std::string line;
   std::string fileType = "fit_analysis";
   std::string fileExt = "*.root";
   std::vector<std::string> files;

   // Final merging
   Bool_t kIsMerging = 0;
   TChain *m_Chain = new TChain("outputTree");

   while (std::getline(file, line)) {
      std::istringstream iss(line);
      if (line.find(fileType) != std::string::npos) {
         std::cout << " Found fit file : " << line << "\n";
         files.push_back(fileFolder + line);

         // files.push_back(line);
      }
   }

   int fileCnt = 0;

   int iEvt = 0;

   // Plot data
   for (auto dataFile : files) {

      TFile rootfile(dataFile.c_str(), "READ");
      if (!rootfile.IsZombie()) {
         if (kIsMerging)
            m_Chain->Add(dataFile.c_str());
         std::cout << " Opening file : " << dataFile << "\n";
         TTree *outputTree = (TTree *)rootfile.Get("outputTree");
         Float_t EFit, AFit, EPRA, APRA, Ex, PhiFit, PhiPRA, xiniPRA, yiniPRA, ziniPRA, xiniFit, yiniFit, ziniFit, IC;
         Int_t ICMult, evMult;
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
         outputTree->SetBranchAddress("evMult", &evMult);

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
         std::vector<Float_t> *particleQVec = 0;
         std::vector<Float_t> *POCAOrbZVec = 0;
         std::vector<Float_t> *firstOrbZVec = 0;
         std::vector<Float_t> *phiOrbZVec = 0;
         std::vector<Float_t> *lengthOrbZVec = 0;
         std::vector<Float_t> *eLossOrbZVec = 0;
         std::vector<Float_t> *brhoVec = 0;
         std::vector<Float_t> *eLossADC = 0;
         std::vector<std::string> *pdgVec = 0;
         std::vector<Int_t> *trackPointsVec = 0;

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
         outputTree->SetBranchAddress("particleQVec", &particleQVec);
         outputTree->SetBranchAddress("POCAOrbZVec", &POCAOrbZVec);
         outputTree->SetBranchAddress("firstOrbZVec", &firstOrbZVec);
         outputTree->SetBranchAddress("phiOrbZVec", &phiOrbZVec);
         outputTree->SetBranchAddress("lengthOrbZVec", &lengthOrbZVec);
         outputTree->SetBranchAddress("eLossOrbZVec", &eLossOrbZVec);
         outputTree->SetBranchAddress("brhoVec", &brhoVec);
         outputTree->SetBranchAddress("eLossADC", &eLossADC);
         outputTree->SetBranchAddress("pdgVec", &pdgVec);
         outputTree->SetBranchAddress("trackPointsVec", &trackPointsVec);

         ++fileCnt;

         Int_t nentries = (Int_t)outputTree->GetEntries();
         for (Int_t i = 0; i < nentries; i++) {

            if (i % 100000 == 0)
               std::cout << " Processing entry : " << i << "/" << nentries << "\n";

            outputTree->GetEntry(i);

            ++iEvt;

            if (ICMult != 1)
               continue;

            // if(evMult !=2)
            // continue;

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

               //   if(ICA<900)
               if (ICA < 500 || ICA > 900)
                  continue;

               for (ICIndex = 0; ICIndex < ICVec->size(); ++ICIndex) {

                  HIC->Fill((*ICVec)[ICIndex]);
                  ICEvsTime->Fill((*ICVec)[ICIndex], (*ICTimeVec)[ICIndex]);
                  ICTimeH->Fill((*ICTimeVec)[ICIndex]);
               }

            } catch (...) {
            }

            // if ((IC > 900 && IC < 1500)) {
            // From std::vector
            assert(EFitVec->size() == AFitVec->size());

            for (auto index = 0; index < EFitVec->size(); ++index) {

               // if((*lengthOrbZVec)[index]<30)
               // continue;

               // if((*brhoVec)[index]>0.8)
               // continue;

               // if((*eLossADC)[index]>50)
               // continue;

               // if((*AFitVec)[index]<8 || (*AFitVec)[index]>19)
               // continue;

               Double_t rad = TMath::Sqrt((*xiniFitXtrVec)[index] * (*xiniFitXtrVec)[index] +
                                          (*yiniFitXtrVec)[index] * (*yiniFitXtrVec)[index]);

               Ang_Ener_PRA->Fill(APRA, EPRA);

               PhiPRAH->Fill(PhiPRA);

               ELossvsBrho->Fill((*eLossADC)[index], (*brhoVec)[index]);

               eventMultH->Fill(evMult);

               // if( ((*xiniFitXtrVec)[index]>0.5 || (*xiniFitXtrVec)[index]<-0.5) )
               // continue;
               // if( ((*yiniFitXtrVec)[index]>0.5 || (*yiniFitXtrVec)[index]<-0.5) )
               //		 continue;

               if ((*POCAXtrVec)[index] < 2000.0) {

                  if ((*ziniFitXtrVec)[index] > -20.0 && (*ziniFitXtrVec)[index] < 200.0) {

                     Double_t angle = (*AFitVec)[index];

                     if ((*fChi2Vec)[index] / (*fNdfVec)[index] > 0.000) { // Chi2

                        if ((*xiniFitVec)[index] > -1000.0) {

                           if (angle > 0 && angle < 180) {

                              if (dataFile.find("sim") != std::string::npos) {
                                 angle = (*AFitVec)[index];
                              }
                              if ((*trackLengthVec)[index] > 0.0) {

                                 if ((*EFitVec)[index] < 100) {

                                    if (rad < 100.5) {

                                       fChi2H->Fill((*fChi2Vec)[index]);
                                       bChi2H->Fill((*bChi2Vec)[index]);
                                       fNdfH->Fill((*fNdfVec)[index]);
                                       bNdfH->Fill((*bNdfVec)[index]);
                                       particleQH->Fill((*particleQVec)[index]);
                                       fChi2NH->Fill((*fChi2Vec)[index]);

                                       // if((*AFitVec)[index]>20.0 && (*EFitVec)[index]>4.0){
                                       // if( ((*xiniFitXtrVec)[index]<2.2 && (*xiniFitXtrVec)[index]>-0.0) &&
                                       // ((*yiniFitXtrVec)[index]<-0.7 && (*yiniFitXtrVec)[index]>-3.7) ){
                                       Ang_Ener->Fill(angle, (*EFitVec)[index]);
                                       HQval->Fill((*ExVec)[index]);
                                       Ang_Ener_Xtr->Fill((angle), (*EFitXtrVec)[index]);
                                       HQval_Xtr->Fill((*ExXtrVec)[index]);
                                       hxpos_fit_Xtr->Fill((*xiniFitXtrVec)[index]);
                                       hypos_fit_Xtr->Fill((*yiniFitXtrVec)[index]);
                                       hzpos_fit_Xtr->Fill((*ziniFitXtrVec)[index]);
                                       x_y_Xtr->Fill((*xiniFitXtrVec)[index], (*yiniFitXtrVec)[index]);
                                       QvsAng_Xtr->Fill((*ExXtrVec)[index], AFit);
                                       POCAXtrH->Fill((*POCAXtrVec)[index]);
                                       tracklengthH->Fill((*trackLengthVec)[index]);
                                       ZposvsEvH->Fill((*ziniFitXtrVec)[index], i * fileCnt);
                                       Double_t rad = TMath::Sqrt((*xiniFitXtrVec)[index] * (*xiniFitXtrVec)[index] +
                                                                  (*yiniFitXtrVec)[index] * (*yiniFitXtrVec)[index]);
                                       ZposvsRad->Fill((*ziniFitXtrVec)[index], rad);
                                       // }// x-y
                                       //}//Energy and angle

                                       // 	  	}//IC

                                       // HIC->Fill(IC);
                                       ICMultH->Fill(ICMult);

                                       // HQval->Fill(Ex);
                                       hxpos_fit->Fill((*xiniFitVec)[index]);
                                       hypos_fit->Fill((*yiniFitVec)[index]);
                                       hzpos_fit->Fill((*ziniFitVec)[index]);

                                       x_y_Fit->Fill((*xiniFitVec)[index], (*yiniFitVec)[index]);

                                       QvsAng->Fill(Ex, AFit);

                                       ZposvsAng->Fill(ziniFit, AFit);
                                       Ang_AngPRA->Fill(AFit, APRA);
                                       zfit_zPRA->Fill(ziniFit, ziniPRA / 10.0);
                                       Phi_PhiPRA->Fill((*PhiFitVec)[index] * TMath::RadToDeg(), (*PhiPRAVec)[index]);
                                       Ang_Phi->Fill(AFit, PhiFit * TMath::RadToDeg());
                                       x_Phi->Fill(xiniFit, PhiFit * TMath::RadToDeg());
                                       y_Phi->Fill(yiniFit, PhiFit * TMath::RadToDeg());

                                       // Excitation energy
                                       Double_t ex_energy_exp =
                                          kine_2b(m_Be10, m_d, m_b, m_B, Ebeam_buff, angle * TMath::DegToRad(),
                                                  (*EFitXtrVec)[index]);
                                       HQval_Xtr_recalc->Fill(ex_energy_exp);

                                       QvsEvent->Fill(ex_energy_exp, iEvt);

                                       // Z correction for the Ex energy
                                       // p0                        =      1.66382   +/-   0.00764931
                                       // p1                        =  -0.00916142   +/-   0.000176617

                                       // p0                        =      1.72831   +/-   0.0239778
                                       // p1                        =     -0.01095   +/-   0.000542567

                                       Double_t p0 = 1.72831;
                                       Double_t p1 = -0.00892026; //-0.01095;

                                       Double_t mFactor = 1.10;
                                       Double_t offSet = 0.0;
                                       Double_t QcorrZ =
                                          ex_energy_exp -
                                          mFactor * ((1.39 - 2.1509) / (87.62 - 0.627)) * ((*ziniFitXtrVec)[index]) +
                                          offSet;
                                       QcorrZ = ex_energy_exp - mFactor * p1 * ((*ziniFitXtrVec)[index]);
                                       HQCorr->Fill(QcorrZ);

                                       for (auto iCorr = 0; iCorr < 10; ++iCorr) {
                                          mFactor = 1.0 + 0.1 * iCorr - 0.5;
                                          Double_t QcorrZL =
                                             ex_energy_exp -
                                             mFactor * ((1.39 - 2.1509) / (87.62 - 0.627)) * ((*ziniFitXtrVec)[index]) +
                                             offSet;

                                          QcorrZL = ex_energy_exp - mFactor * p1 * ((*ziniFitXtrVec)[index]);

                                          HQCorrArray[iCorr]->Fill(QcorrZL);
                                       }

                                       QvsChi2->Fill(ex_energy_exp, (*fChi2Vec)[index] / (*fNdfVec)[index]);
                                       QvsXpos->Fill(ex_energy_exp, (*xiniFitXtrVec)[index]);
                                       QvsZpos->Fill(ex_energy_exp, (*ziniFitXtrVec)[index]);
                                       QcorrvsZpos->Fill(QcorrZ, (*ziniFitXtrVec)[index]);

                                       QvsTrackLengthH->Fill(QcorrZ, (*trackLengthVec)[index]);

                                       // First Orbit
                                       if ((*phiOrbZVec)[index] > 5.0 && (*phiOrbZVec)[index] < 8.0) {
                                          Double_t OrbZ = (*firstOrbZVec)[index];
                                          if ((*lengthOrbZVec)[index] > 20) {
                                             fOrbZvsfOrbLength->Fill((*firstOrbZVec)[index] - (*ziniFitXtrVec)[index],
                                                                     (*lengthOrbZVec)[index]);
                                             PhiOrbZH->Fill((*phiOrbZVec)[index]);
                                             fOrbLengthvsEFit->Fill((*lengthOrbZVec)[index], (*EFitVec)[index]);
                                             fOrbZvsEFit->Fill((*firstOrbZVec)[index] - (*ziniFitXtrVec)[index],
                                                               (*EFitVec)[index]);
                                             fOrbZvsEx->Fill((*firstOrbZVec)[index], QcorrZ);
                                             fOrbZvsZ->Fill((*firstOrbZVec)[index], (*ziniFitXtrVec)[index]);
                                             HQCorrOrbZ->Fill(QcorrZ);
                                             fOrbZvsAFit->Fill((*firstOrbZVec)[index] - (*ziniFitXtrVec)[index],
                                                               (*AFitVec)[index]);
                                             fOrbZvsMomLoss->Fill((*firstOrbZVec)[index] - (*ziniFitXtrVec)[index],
                                                                  (*eLossOrbZVec)[index]);
                                             fOrbLengthvsMomLoss->Fill((*lengthOrbZVec)[index], (*eLossOrbZVec)[index]);
                                          }
                                       }

                                       if ((*ziniFitXtrVec)[index] > 0.0 && (*ziniFitXtrVec)[index] < 100.0) {
                                          Int_t zIndex = (Int_t)floor((*ziniFitXtrVec)[index] / 10);
                                          // std::cout<<" zIndex "<<zIndex<<"\n";
                                          ExZ[zIndex]->Fill(ex_energy_exp);
                                       }

                                       if (cutGS->IsInside(angle, (*EFitVec)[index])) {
                                          HQval_Xtr_recalc_cutgs->Fill(ex_energy_exp);
                                       }

                                       for (auto iEb = 0; iEb < 300; ++iEb) {
                                          double Qdep = kine_2b(m_Be10, m_d, m_b, m_B, iEb, angle * TMath::DegToRad(),
                                                                (*EFitVec)[index]);
                                          QvsEb->Fill(Qdep, iEb);
                                       }

                                       // HQval->Fill(Ex);
                                    }
                                 } // Xini
                              }    // Rad
                           }       // Z vertex
                                   //}//X-Y
                        }          // Chi2
                     }             // POCA
                  }                // Angle
               }                   // Track length
            }                      // Energy
         }
      }
   }

   // Merging
   if (kIsMerging)
      m_Chain->Merge("final.root", "C");

   // Adding kinematic lines
   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "O16_aa_gs.txt";
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

   fileKine = "Be10pp_el.txt";
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

   fileKine = "O16_aa_gs_11.txt";
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
   cutGS->Draw("l");
   c1->Modified();

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
   c3->cd(2);
   PhiPRAH->Draw();

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
   HIC->Draw();
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

   TCanvas *c6 = new TCanvas();
   c6->Divide(3, 3);
   c6->Draw();
   c6->cd(1);
   POCAXtrH->Draw();
   c6->cd(2);
   hxpos_fit->Draw();
   c6->cd(3);
   hypos_fit->Draw();
   c6->cd(4);
   hzpos_fit->Draw();
   c6->cd(5);
   tracklengthH->Draw();
   c6->cd(6);
   ZposvsEvH->Draw();
   c6->cd(7);
   x_y_Fit->Draw("zcol");
   c6->cd(8);
   ELossvsBrho->Draw();
   c6->cd(9);
   eventMultH->Draw();

   TCanvas *c7 = new TCanvas();
   c7->Divide(2, 2);
   c7->Draw();
   c7->cd(1);
   HQval_Xtr_recalc->Draw();
   c7->cd(2);
   HQval_Xtr_recalc_cutgs->Draw();
   c7->cd(3);
   QvsEb->Draw("zcol");
   c7->cd(4);
   ZposvsRad->Draw("zcol");

   /*TCanvas *cEx = new TCanvas();
   cEx->Divide(2, 5);
   for (auto cIndex = 0; cIndex < 10; ++cIndex) {
      cEx->cd(cIndex + 1);
      // ExZ[cIndex]->Draw();
      HQCorrArray[cIndex]->Draw();
      }*/

   TCanvas *cChi = new TCanvas();
   cChi->Divide(3, 3);
   cChi->Draw();
   cChi->cd(1);
   fChi2H->Draw();
   cChi->cd(2);
   bChi2H->Draw();
   cChi->cd(3);
   fNdfH->Draw();
   cChi->cd(4);
   bNdfH->Draw();
   cChi->cd(5);
   fChi2NH->Draw();
   cChi->cd(6);
   QvsChi2->Draw("zcol");
   cChi->cd(7);
   particleQH->Draw();
   // cChi->cd(8);

   TCanvas *cOrb = new TCanvas();
   cOrb->Divide(3, 4);
   cOrb->Draw();
   cOrb->cd(1);
   PhiOrbZH->Draw();
   cOrb->cd(2);
   fOrbLengthvsEFit->Draw();
   cOrb->cd(3);
   fOrbZvsEFit->Draw();
   cOrb->cd(4);
   fOrbZvsfOrbLength->Draw();
   cOrb->cd(5);
   fOrbZvsEx->Draw();
   cOrb->cd(6);
   fOrbZvsZ->Draw();
   cOrb->cd(7);
   HQCorrOrbZ->Draw();
   cOrb->cd(8);
   fOrbZvsAFit->Draw();
   cOrb->cd(9);
   fOrbZvsMomLoss->Draw();
   cOrb->cd(10);
   fOrbLengthvsMomLoss->Draw();

   TCanvas *c8 = new TCanvas();
   c8->Divide(2, 2);
   c8->Draw();
   c8->cd(1);
   HQCorr->Draw();
   c8->cd(2);
   QcorrvsZpos->Draw();
   c8->cd(3);
   QvsEvent->Draw();
   c8->cd(4);
   QvsTrackLengthH->Draw();
}
