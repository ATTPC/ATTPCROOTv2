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

void plotFit_a1954(std::string fileFolder = "data_56_68/")
{

   std::ofstream outputFileEvents("list_of_events.txt");

   gStyle->SetNumberContours(256);
   gStyle->SetPalette(112);

   // DWBA calculations
   Double_t sigmaDWBA0 = 0;
   Double_t sigmaDWBA1 = 0;
   Double_t sigmaDWBA2 = 0;
   Double_t altXS = 0;
   Double_t angle = 0;
   TGraphErrors *gDWBA0 = new TGraphErrors();
   gDWBA0->SetMarkerStyle(20);
   gDWBA0->SetMarkerSize(1.5);
   gDWBA0->SetLineWidth(3);
   TGraphErrors *gDWBA1 = new TGraphErrors();
   gDWBA1->SetMarkerStyle(20);
   gDWBA1->SetMarkerSize(1.5);
   gDWBA1->SetLineWidth(3);
   TGraphErrors *gDWBA2 = new TGraphErrors();
   gDWBA2->SetMarkerStyle(20);
   gDWBA2->SetMarkerSize(1.5);
   gDWBA2->SetLineWidth(3);
   gDWBA2->SetLineColor(kRed);

   std::ifstream dwbaFile("kinematics_a1954/C14_pp_elastic_DWBA.txt");

   std::string linebuff;
   // Read header and zero degree
   // for(auto i=0;i<5;++i)
   //  std::getline(dwbaFile, linebuff);

   while (!dwbaFile.eof()) {
      std::getline(dwbaFile, linebuff);
      std::istringstream iss(linebuff);
      // iss >> angle >> sigmaDWBA0 >> sigmaDWBA1 >> sigmaDWBA2;
      iss >> angle >> sigmaDWBA0;
      gDWBA0->SetPoint(gDWBA0->GetN(), angle, sigmaDWBA0);

      // gDWBA1->SetPoint(gDWBA1->GetN(), angle, sigmaDWBA1);
      // gDWBA2->SetPoint(gDWBA2->GetN(), angle, sigmaDWBA2);
    }

    dwbaFile.close();

    // Alt analysis
    /*std::ifstream altFile("kinematics_a1954/combe10dd_diffxs.txt");

    TGraphErrors *gAltAnalysis = new TGraphErrors();
    gAltAnalysis->SetMarkerStyle(20);
    gAltAnalysis->SetMarkerSize(1.5);
    gAltAnalysis->SetMarkerColor(kRed);

    while (!altFile.eof()) {
       std::getline(altFile, linebuff);
       std::istringstream iss(linebuff);
       iss >> angle >> altXS;
       gAltAnalysis->SetPoint(gAltAnalysis->GetN(), angle, altXS);
    }

    altFile.close();*/

    // Data histograms
    TH2F *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 179, 1000, 0, 100.0);
    TH2F *Ang_Ener_PRAC = new TH2F("Ang_Ener_PRAC", "Ang_Ener_PRAC", 1000, 0, 100, 1000, 0, 10.0);
    TH1F *HQval = new TH1F("HQval", "HQval", 600, -5, 55);
    TH1F *HQvalPRA = new TH1F("HQvalPRA", "HQvalPRA", 600, -5, 55);
    TH1F *HIC = new TH1F("HIC", "HIC", 1000, 0, 4095);

    TH2F *Ang_Ener_Xtr = new TH2F("Ang_Ener_Xtr", "Ang_Ener_Xtr", 720, 0, 179, 1000, 0, 100.0);
    TH1F *HQval_Xtr = new TH1F("HQval_Xtr", "HQval_Xtr", 600, -5, 55);
    TH1F *HQCorr = new TH1F("HQCorr", "HQCorr", 600, -5, 55);
    TH1F *HQCorrArray[10];

    TH1F *HQval_Xtr_recalc = new TH1F("HQval_Xtr_recalc", "HQval_Xtr_recalc", 1200, -5, 55);
    TH1F *HQval_Xtr_recalc_cutps = new TH1F("HQval_Xtr_recalc_cutps", "HQval_Xtr_recalc_cutps", 1200, -5, 55);

    TH2F *QvsAng = new TH2F("QvsAng", "QvsAng", 1000, -5, 25, 720, 0, 179);
    TH2F *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 200, -100, 100);
    TH2F *QcorrvsZpos = new TH2F("QcorrvsZpos", "QcorrvsZpos", 1000, -10, 10, 200, -100, 100);
    TH2F *ZposvsAng = new TH2F("ZposvsAng", "ZposvsAng", 200, -100, 100, 720, 0, 179);
    TH2F *QvsXpos = new TH2F("QvsXpos", "QvsXpos", 1000, -10, 10, 100, -10, 10);

    TH2F *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);

    TH2F *QvsTrackLengthH = new TH2F("QvsTrackLengthH", "QvsTrackLengthH", 1000, -10, 50, 1000, 0, 1000);

    TH2F *QvsAng_Xtr = new TH2F("QvsAng_Xtr", "QvsAng_Xtr", 1000, -10, 10, 720, 0, 179);

    TH1F *hxpos_fit = new TH1F("hxpos_fit", "hxpos_fit", 100, -10, 10);
    TH1F *hypos_fit = new TH1F("hypos_fit", "hypos_fit", 100, -10, 10);
    TH1F *hzpos_fit = new TH1F("hzpos_fit", "hzpos_fit", 200, -100, 100);

    TH1F *hxpos_fit_Xtr = new TH1F("hxpos_fit_Xtr", "hxpos_fit_Xtr", 100, -10, 10);
    TH1F *hypos_fit_Xtr = new TH1F("hypos_fit_Xtr", "hypos_fit_Xtr", 100, -10, 10);
    TH1F *hzpos_fit_Xtr = new TH1F("hzpos_fit_Xtr", "hzpos_fit_Xtr", 300, -100, 200);

    // PRA
    TH2F *Ang_Ener_PRA = new TH2F("Ang_Ener_PRA", "Ang_Ener_PRA", 720, 0, 179, 1000, 0, 100.0);
    TH2F *Ang_Ener_PRA_Cond = new TH2F("Ang_Ener_PRA_Cond", "Ang_Ener_PRA_Cond", 720, 0, 179, 1000, 0, 100.0);
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

    TH2F *ELossvsBrho = new TH2F("ELossvsBrho", "ELossvsBrho", 4000, 0, 4000, 1000, 0, 3);
    TH2F *ELossvsBrhoZoom = new TH2F("ELossvsBrhoZoom", "ELossvsBrhoZoom", 4000, 0, 20000, 1000, 0, 3);
    TH2F *dedxvsBrho = new TH2F("dedxvsBrho", "dedxvsBrho", 4000, 0, 10000, 1000, 0, 3);
    TH2F *dedxvsBrhoCond = new TH2F("dedxvsBrhoCond", "dedxvsBrhoCond", 4000, 0, 10000, 1000, 0, 3);
    TH2F *dedxvsBrhoZoom = new TH2F("dedxvsBrhoZoom", "dedxvsBrhoZoom", 4000, 0, 1000, 1000, 0, 3);

    TH2F *multvsnumpoints = new TH2F("multvsnumpoints", "multvsnumpoints", 10, 0, 10, 500, 0, 500);

    for (auto iHist = 0; iHist < 10; ++iHist)
       ExZ[iHist] = new TH1F(Form("ExZ[%i]", iHist), Form("ExZ[%i]", iHist), 1000, -2, 18);

    for (auto iHist = 0; iHist < 10; ++iHist)
       HQCorrArray[iHist] = new TH1F(Form("HQCorrArray[%i]", iHist), Form("HQCorrArray[%i]", iHist), 600, -5, 55);

    TH2F *QvsEvent = new TH2F("QvsEvent", "QvsEvent", 1000, -10, 10, 1000, 0, 1000);
    TH2F *QvsMult = new TH2F("QvsMult", "QvsMult", 1000, -10, 10, 10, 0, 10);

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

    // Binary
    TH2F *hARecvsASca = new TH2F("hARecvsASca", "hARecvsASca", 1000, 0, 180, 1000, 0, 180);

    // Cut 14C proton gate
    TCutG *cutp = new TCutG("cutp", 18);
    cutp->SetVarX("ELossvsBrhoZoom");
    cutp->SetVarY("");
    cutp->SetTitle("Graph");
    cutp->SetFillStyle(1000);
    cutp->SetPoint(0, 181.1004, 1.092132);
    cutp->SetPoint(1, 1011.636, 0.556248);
    cutp->SetPoint(2, 2235.584, 0.4222769);
    cutp->SetPoint(3, 4399.348, 0.3329628);
    cutp->SetPoint(4, 9207.713, 0.1900603);
    cutp->SetPoint(5, 11917.88, 0.1811289);
    cutp->SetPoint(6, 16070.56, 0.1543347);
    cutp->SetPoint(7, 19130.43, 0.1543347);
    cutp->SetPoint(8, 19829.83, 0.1543347);
    cutp->SetPoint(9, 19960.96, 0.09181484);
    cutp->SetPoint(10, 19807.97, 0.002500774);
    cutp->SetPoint(11, 7109.517, 0.0382264);
    cutp->SetPoint(12, 2694.564, 0.06502062);
    cutp->SetPoint(13, 530.7996, 0.2257859);
    cutp->SetPoint(14, 49.96314, 0.6455621);
    cutp->SetPoint(15, 28.10694, 0.8509844);
    cutp->SetPoint(16, 115.5318, 1.07427);
    cutp->SetPoint(17, 181.1004, 1.092132);

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

    Double_t Ebeam_buff = 162.0; // 169.5; // 167.5;
    Double_t m_b;
    Double_t m_B;

    m_b = m_p;
    m_B = m_C14;

    // Differential cross sections
    Double_t sigmaLab0[360] = {0.0};
    Double_t sigmaCM0[360] = {0.0};
    /*TGraphErrors *gsigmaLab0 = new TGraphErrors();
    gsigmaLab0->SetMarkerStyle(20);
    gsigmaLab0->SetMarkerSize(1.5);
    TGraphErrors *gsigmaCM0 = new TGraphErrors();
    gsigmaCM0->SetMarkerStyle(20);
    gsigmaCM0->SetMarkerSize(1.5);*/

    Double_t sigmaCM0Corr[360];
    std::fill_n(sigmaCM0Corr, 360, 1.0);

    Double_t sigmaLab1[360] = {0.0};
    Double_t sigmaCM1[360] = {0.0};
    TGraphErrors *gsigmaLab1 = new TGraphErrors();
    gsigmaLab1->SetMarkerStyle(21);
    gsigmaLab1->SetMarkerSize(1.5);
    gsigmaLab1->SetMarkerColor(kRed);
    TGraphErrors *gsigmaCM1 = new TGraphErrors();
    gsigmaCM1->SetMarkerStyle(21);
    gsigmaCM1->SetMarkerSize(1.5);
    gsigmaCM1->SetMarkerColor(kRed);

    TH1F *hsigmaCM1 = new TH1F("hsigmaCM1", "hsigmaCM1", 360, 0, 360);

    Double_t sigmaLab2[360] = {0.0};
    Double_t sigmaCM2[360] = {0.0};
    /*TGraphErrors *gsigmaLab2 = new TGraphErrors();
    gsigmaLab2->SetMarkerStyle(22);
    gsigmaLab2->SetMarkerSize(1.5);
    gsigmaLab2->SetMarkerColor(kBlue);
    TGraphErrors *gsigmaCM2 = new TGraphErrors();
    gsigmaCM2->SetMarkerStyle(22);
    gsigmaCM2->SetMarkerSize(1.5);
    gsigmaCM2->SetMarkerColor(kBlue);*/

    Double_t sigmaLab3[360] = {0.0};
    Double_t sigmaCM3[360] = {0.0};
    TGraphErrors *gsigmaLab3 = new TGraphErrors();
    gsigmaLab3->SetMarkerStyle(23);
    gsigmaLab3->SetMarkerSize(1.5);
    gsigmaLab3->SetMarkerColor(kGreen);
    TGraphErrors *gsigmaCM3 = new TGraphErrors();
    gsigmaCM3->SetMarkerStyle(23);
    gsigmaCM3->SetMarkerSize(1.5);
    gsigmaCM3->SetMarkerColor(kGreen);

    // Find every valid file
    std::string command = "find ./" + fileFolder + " -maxdepth 1 -printf \"%f\n\" >test.txt";
    std::system(command.c_str()); // execute the UNIX command "ls -l
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
         Int_t ICMult, evMult, praMult;
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
         outputTree->SetBranchAddress("praMult", &praMult);

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
         std::vector<Float_t> *dEdxADC = 0;
         std::vector<std::string> *pdgVec = 0;
         std::vector<Int_t> *trackPointsVec = 0;
         std::vector<Bool_t> *fitConvergedVec = 0;

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
         outputTree->SetBranchAddress("dEdxADC", &dEdxADC);
         outputTree->SetBranchAddress("pdgVec", &pdgVec);
         outputTree->SetBranchAddress("trackPointsVec", &trackPointsVec);
         outputTree->SetBranchAddress("fitConvergedVec", &fitConvergedVec);

         ++fileCnt;

         Int_t nentries = (Int_t)outputTree->GetEntries();
         for (Int_t i = 0; i < nentries; i++) {

            if (i % 100000 == 0)
               std::cout << " Processing entry : " << i << "/" << nentries << "\n";

            outputTree->GetEntry(i);

            ++iEvt;

            // if (ICMult > 3)
            // continue;

            // if(evMult !=2)
            //   continue;

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

               //  if (ICA < 500 || ICA > 900)
               // continue;

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

            /*std::cout<<" Event mult : "<<evMult<<"\n";
                    std::cout<<" AFit size : "<<AFitVec->size()<<"\n";
                    for(auto inxaf =0;inxaf<AFitVec->size();++inxaf)
                 std::cout<<" Angle "<<(*AFitVec)[inxaf]<<"  - ";

                 std::cout<<"\n";*/

            // Get the track with maximum angle
            auto itMax =
               std::max_element(APRAVec->begin(), APRAVec->end(), [](const auto &a, const auto &b) { return b > a; });
            Int_t maxAIndex = std::distance(APRAVec->begin(), itMax);
            ;

            // Calculate mean distance to vertex (0,0) of each track
            Double_t meanDist = 0.0;
            for (auto index = 0; index < xiniPRAVec->size(); ++index) {
               Double_t x = (*xiniPRAVec)[index];
               Double_t y = (*yiniPRAVec)[index];
               Double_t z = (*ziniPRAVec)[index];
               meanDist += TMath::Sqrt(x * x + y * y);
            }
            meanDist /= xiniPRAVec->size();

            for (auto index = 0; index < EFitVec->size(); ++index) {

               if (index != maxAIndex)
                  continue;

               eventMultH->Fill(evMult);
               multvsnumpoints->Fill(evMult, (*trackPointsVec)[index]);

               Ang_Ener_PRA->Fill(APRA, EPRA);
               PhiPRAH->Fill(PhiPRA);

               // if((*lengthOrbZVec)[index]<30)
               // continue;

               // if((*brhoVec)[index]>0.8)
               // continue;

               // if((*eLossADC)[index]<1000)
               // continue;

               // if ((*AFitVec)[index] < 0 || (*AFitVec)[index] > 90)
               // continue;

               // Particle ID
               ELossvsBrho->Fill((*eLossADC)[index], (*brhoVec)[index]);
               ELossvsBrhoZoom->Fill((*eLossADC)[index], (*brhoVec)[index]);
               // if (!cutT->IsInside((*eLossADC)[index], (*brhoVec)[index]) && !cutD->IsInside((*eLossADC)[index],
               // (*brhoVec)[index])) {
               dedxvsBrho->Fill((*dEdxADC)[index], (*brhoVec)[index]);
               dedxvsBrhoZoom->Fill((*dEdxADC)[index], (*brhoVec)[index]);
               //}

               if (!cutp->IsInside((*eLossADC)[index], (*brhoVec)[index])) // Proton gate
                  continue;
               /*if (!cutD->IsInside((*eLossADC)[index], (*brhoVec)[index])) // particleID
                  continue;*/

               Ang_Ener_PRA_Cond->Fill(APRA, EPRA);

               // if(!cutDEDX->IsInside((*dEdxADC)[index], (*brhoVec)[index]))
               // continue;

               // if ((*dEdxADC)[index] < 3000) // particleID
               // continue;

               /*   if ((*trackLengthVec)[index] < 65.0)
                     continue;*/

               // if ((*fitConvergedVec)[index] == 0)
               // continue;

               // if((*trackPointsVec)[index]<20)
               // continue;

               // if (evMult != 3)
               //  continue;

               // if ((*POCAXtrVec)[index] > 2000.0)
               // continue;

               if ((*ziniFitVec)[index] < 20.0 || (*ziniFitVec)[index] > 60.0)
                  continue;

               // if((*AFitVec)[index]<50 || (*AFitVec)[index]>70)
               // continue;

               if ((*EFitVec)[index] < 0 || (*EFitVec)[index] > 15)
                  continue;

               /*     if ((*fChi2Vec)[index] / (*fNdfVec)[index] < 0.000)
                      continue;


                         if ((*xiniFitVec)[index] < -1000.0)
                         continue;*/

               dedxvsBrhoCond->Fill((*dEdxADC)[index], (*brhoVec)[index]);

               Double_t angle = (*AFitVec)[index];
               if (dataFile.find("sim") != std::string::npos) {
                  angle = (*AFitVec)[index];
               }

               if (index == 0 && AFitVec->size() > 1) {
                  hARecvsASca->Fill((*AFitVec)[0], (*AFitVec)[1]);
               }

               // Excitation energy
               auto [ex_energy_exp, theta_cm] =
                  kine_2b(m_C14, m_p, m_b, m_B, Ebeam_buff, angle * TMath::DegToRad(), (*EFitVec)[index]);
               auto [ex_energy_exp_xtr, theta_cm_xtr] =
                  kine_2b(m_C14, m_p, m_b, m_B, Ebeam_buff, angle * TMath::DegToRad(), (*EFitXtrVec)[index]);

               auto [exPRA, thetacmPRA] = kine_2b(m_C14, m_p, m_b, m_B, Ebeam_buff, APRA * TMath::DegToRad(), EPRA);

               // Excitation energy correction
               Double_t p0 = 0.0;    //-3.048;
               Double_t p1 = 0.002;  // 0.0513295;
               Double_t mFactor = 1.00;
               Double_t offSet = 0.0;
               Double_t QcorrZ = 0.0;

               // Ad hoc correction (to be investigated)
               if (ex_energy_exp > 5.0)
                  QcorrZ = ex_energy_exp - mFactor * p1 * ((*ziniFitXtrVec)[index]) - p0;
               else
                  QcorrZ = ex_energy_exp;


               // if(QcorrZ<7.0 || QcorrZ>8.0)
               // continue;

               // if (QcorrZ < 5.5 || QcorrZ > 6.8)
               // continue;

               // Chi2
               fChi2H->Fill((*fChi2Vec)[index]);
               bChi2H->Fill((*bChi2Vec)[index]);
               fNdfH->Fill((*fNdfVec)[index]);
               bNdfH->Fill((*bNdfVec)[index]);
               fChi2NH->Fill((*fChi2Vec)[index]);

               // Kinematics
               Ang_Ener->Fill(angle, (*EFitVec)[index]);
               Ang_Ener_Xtr->Fill((angle), (*EFitXtrVec)[index]);

               if (exPRA > 8.5 && exPRA < 10.0)
                  Ang_Ener_PRAC->Fill(exPRA, ex_energy_exp);

               // List of events
               outputFileEvents << dataFile << " - Ev. : " << i << " - PRA.Mult : " << praMult
                                << " - Ev.Mult : " << evMult << " - Max.PRA : " << (*APRAVec)[index]
                                << " - Max.Fit : " << (*AFitVec)[index] << " - Q.val : " << ex_energy_exp
                                << " - Track points : " << (*trackPointsVec)[index] << "\n";

               HQvalPRA->Fill(exPRA);
               HQval->Fill(ex_energy_exp);
               HQval_Xtr->Fill(ex_energy_exp_xtr);
               HQval_Xtr_recalc->Fill(ex_energy_exp);

               /*for (auto iCorr = 0; iCorr < 10; ++iCorr) {
                  mFactor = 1.0 + 0.1 * iCorr - 0.5;
                  Double_t QcorrZL =
                     ex_energy_exp -
                     mFactor * ((1.39 - 2.1509) / (87.62 - 0.627)) * ((*ziniFitXtrVec)[index]) +
                     offSet;

                  QcorrZL = ex_energy_exp - mFactor * p1 * ((*ziniFitXtrVec)[index]);

                  HQCorrArray[iCorr]->Fill(QcorrZL);
}*/

               // Excitation energy correlations
               QvsChi2->Fill(ex_energy_exp, (*fChi2Vec)[index] / (*fNdfVec)[index]);
               QvsXpos->Fill(ex_energy_exp, (*xiniFitXtrVec)[index]);
               QvsZpos->Fill(ex_energy_exp, (*ziniFitXtrVec)[index]);
               QcorrvsZpos->Fill(QcorrZ, (*ziniFitXtrVec)[index]);
               QvsTrackLengthH->Fill(QcorrZ, (*trackLengthVec)[index]);
               //--------------

               // Positions
               hxpos_fit->Fill((*xiniFitVec)[index]);
               hypos_fit->Fill((*yiniFitVec)[index]);
               hzpos_fit->Fill((*ziniFitVec)[index]);
               x_y_Fit->Fill((*xiniFitVec)[index], (*yiniFitVec)[index]);
               hxpos_fit_Xtr->Fill((*xiniFitXtrVec)[index]);
               hypos_fit_Xtr->Fill((*yiniFitXtrVec)[index]);
               hzpos_fit_Xtr->Fill((*ziniFitXtrVec)[index]);
               x_y_Xtr->Fill((*xiniFitXtrVec)[index], (*yiniFitXtrVec)[index]);

               // Particle and track
               // QvsAng_Xtr->Fill((*ExXtrVec)[index], AFit);
               POCAXtrH->Fill((*POCAXtrVec)[index]);
               tracklengthH->Fill((*trackLengthVec)[index]);

               Double_t rad = TMath::Sqrt((*xiniFitXtrVec)[index] * (*xiniFitXtrVec)[index] +
                                          (*yiniFitXtrVec)[index] * (*yiniFitXtrVec)[index]);
               ZposvsRad->Fill((*ziniFitXtrVec)[index], rad);
               particleQH->Fill((*particleQVec)[index]);

               // Ion Chamber
               ICMultH->Fill(ICMult);

               // Correlations
               QvsAng->Fill(ex_energy_exp, AFit);
               ZposvsAng->Fill(ziniFit, AFit);
               Ang_AngPRA->Fill(AFit, APRA);
               zfit_zPRA->Fill(ziniFit, ziniPRA / 10.0);
               Phi_PhiPRA->Fill((*PhiFitVec)[index] * TMath::RadToDeg(), (*PhiPRAVec)[index]);
               Ang_Phi->Fill(AFit, PhiFit * TMath::RadToDeg());
               x_Phi->Fill(xiniFit, PhiFit * TMath::RadToDeg());
               y_Phi->Fill(yiniFit, PhiFit * TMath::RadToDeg());

               // Angular distributions
               Double_t cm_angle = 0.0;
               // Ground state
               if (QcorrZ > -2.0 && QcorrZ < 2.0) {

                  theta_cm = -2.0 * angle + 179.76; // g.s.
                  Int_t index = angle;
                  ++sigmaLab0[index];
                  Int_t indexCM = theta_cm;
                  ++sigmaCM0[indexCM];
               }

               // Group of 3 states
               if (QcorrZ > 6.4 && QcorrZ < 7.7) { // 9.2 MeV

                  // if((*EFitVec)[index]<2.75

                  Int_t index = angle;
                  ++sigmaLab1[index];
                  Int_t indexCM = theta_cm;
                  ++sigmaCM1[indexCM];
                  cm_angle = theta_cm;
                  // std::cout<<cm_angle<<"\n";
               }

               // 8.3 MeV state
               if (QcorrZ > 8.0 && QcorrZ < 8.7) {

                  Int_t index = angle;
                  ++sigmaLab2[index];
                  Int_t indexCM = theta_cm;
                  ++sigmaCM2[indexCM];
               }

               if (QcorrZ > 7.0 && QcorrZ < 8.5) {

                  Double_t gamma = 1.405;

                  /*  if((*EFitVec)[index]<6.25)
                      Double_t theta_cm = TMath::RadToDeg()*TMath::ASin(gamma * TMath::Sin(angle*TMath::DegToRad())
                    )-angle
                    ;//low value else theta_cm = 180.0-(angle+TMath::RadToDeg()*TMath::ASin(gamma *
                    TMath::Sin(angle*TMath::DegToRad())));//; high value*/

                  //  std::cout<<theta_cm<<" "<<angle<<"\n";

                  //  if(theta_cm>0 && theta_cm<180){
                  Int_t index = angle;
                  ++sigmaLab3[index];
                  Int_t indexCM = theta_cm;
                  ++sigmaCM3[indexCM];
                  //   }
               }

               QvsMult->Fill(QcorrZ, evMult);

               if (cm_angle < 20.0 || cm_angle > 30)
                  continue;

               hsigmaCM1->Fill(cm_angle);
               HQCorr->Fill(QcorrZ);

               // QvsEvent->Fill(ex_energy_exp, iEvt);

               // First Orbit
               if ((*phiOrbZVec)[index] > 0.0 && (*phiOrbZVec)[index] < 100.0) {
                  Double_t OrbZ = (*firstOrbZVec)[index];
                  if ((*lengthOrbZVec)[index] > 0) {
                     fOrbZvsfOrbLength->Fill((*firstOrbZVec)[index] - (*ziniFitXtrVec)[index], (*lengthOrbZVec)[index]);
                     PhiOrbZH->Fill((*phiOrbZVec)[index]);
                     fOrbLengthvsEFit->Fill((*lengthOrbZVec)[index], (*EFitVec)[index]);
                     fOrbZvsEFit->Fill((*firstOrbZVec)[index] - (*ziniFitXtrVec)[index], (*EFitVec)[index]);
                     fOrbZvsEx->Fill((*firstOrbZVec)[index], QcorrZ);
                     fOrbZvsZ->Fill((*firstOrbZVec)[index], (*ziniFitXtrVec)[index]);
                     HQCorrOrbZ->Fill(QcorrZ);
                     fOrbZvsAFit->Fill((*firstOrbZVec)[index] - (*ziniFitXtrVec)[index], (*AFitVec)[index]);
                     fOrbZvsMomLoss->Fill((*firstOrbZVec)[index] - (*ziniFitXtrVec)[index], (*eLossOrbZVec)[index]);
                     fOrbLengthvsMomLoss->Fill((*lengthOrbZVec)[index], (*eLossOrbZVec)[index]);
                  }
               }

               // Selection of first orbit
               if ((*ziniFitXtrVec)[index] > 0.0 && (*ziniFitXtrVec)[index] < 100.0) {
                  Int_t zIndex = (Int_t)floor((*ziniFitXtrVec)[index] / 10);
                  // std::cout<<" zIndex "<<zIndex<<"\n";
                  ExZ[zIndex]->Fill(ex_energy_exp);
               }

               /*if (cutpS->IsInside(angle, (*EFitVec)[index])) {
                  HQval_Xtr_recalc_cutps->Fill(ex_energy_exp);
               }*/

               // Excitation energy vs Beam energy
               for (auto iEb = 0; iEb < 300; ++iEb) {
                  auto [Qdep, theta_cm_qdep] =
                     kine_2b(m_C14, m_p, m_b, m_B, iEb, angle * TMath::DegToRad(), (*EFitVec)[index]);
                  QvsEb->Fill(Qdep, iEb);
               }

               // HQval->Fill(Ex);
            }
         }
      }
   }

   // Output file for cross sections
   std::ofstream outputXSFile("diff_xs.txt");

   outputXSFile << " Theta CM "
                << " - "
                << " gs "
                << " - "
                << " first "
                << " - "
                << " second "
                << " - "
                << " third "
                << "\n";

   // Efficeincy correction factors for different excited states

   auto fcorr0 =
      new TF1("fcorr0", "[0] + [1]*x + [2]*TMath::Power(x,2) + [3]*TMath::Power(x,3) + [4]*TMath::Power(x,4) ", 0, 180);

   // Works with linear labtocm transformation
   fcorr0->SetParameter(0, -0.393359);
   fcorr0->SetParameter(1, 0.03903);
   fcorr0->SetParameter(2, -0.000353706);
   fcorr0->SetParameter(3, 7.99494e-07);
   fcorr0->SetParameter(4, 0);

   auto fcorr1 =
      new TF1("fcorr1", "[0] + [1]*x + [2]*TMath::Power(x,2) + [3]*TMath::Power(x,3) + [4]*TMath::Power(x,4) ", 0, 180);
   fcorr1->SetParameter(0, 9.38768);
   fcorr1->SetParameter(1, -0.752249);
   fcorr1->SetParameter(2, 0.0224853);
   fcorr1->SetParameter(3, -0.000281328);
   fcorr1->SetParameter(4, 1.26164e-06);

   auto fcorr3 =
      new TF1("fcorr3", "[0] + [1]*x + [2]*TMath::Power(x,2) + [3]*TMath::Power(x,3) + [4]*TMath::Power(x,4) ", 0, 180);
   fcorr3->SetParameter(0, 2.31713);
   fcorr3->SetParameter(1, -0.0948616);
   fcorr3->SetParameter(2, 0.00176076);
   fcorr3->SetParameter(3, 1.00102e-05);
   fcorr3->SetParameter(4, 0);

   // Diff xs graph
   Double_t beamIntensity = 59855710.0;
   Double_t scale0 = 1.0; //* (2.0 * TMath::Pi()) / (beamIntensity * 3.16E21 * 1E-27);
   Double_t scale1 = 1.0; // 2.0 * 1.0 * (2.0 * TMath::Pi()) / (beamIntensity * 3.16E21 * 1E-27);
   Double_t scale2 = 1.0;
   Double_t scale3 = 1.0;

   TGraphErrors *gsigmaLab0 = new TGraphErrors();
   gsigmaLab0->SetMarkerStyle(20);
   gsigmaLab0->SetMarkerSize(1.5);
   TGraphErrors *gsigmaCM0 = new TGraphErrors();
   gsigmaCM0->SetMarkerStyle(20);
   gsigmaCM0->SetMarkerSize(1.5);

   TGraphErrors *gsigmaLab2 = new TGraphErrors();
   gsigmaLab2->SetMarkerStyle(22);
   gsigmaLab2->SetMarkerSize(1.5);
   gsigmaLab2->SetMarkerColor(kBlue);
   TGraphErrors *gsigmaCM2 = new TGraphErrors();
   gsigmaCM2->SetMarkerStyle(22);
   gsigmaCM2->SetMarkerSize(1.5);
   gsigmaCM2->SetMarkerColor(kBlue);

   for (auto ig = 1; ig < 360; ++ig)

   {

      gsigmaLab0->SetPoint(ig, ig, sigmaLab0[ig]);
      gsigmaLab0->SetPointError(ig, 0, TMath::Sqrt(sigmaLab0[ig]));
      gsigmaCM0->SetPoint(ig, ig,
                          sigmaCM0[ig] * scale0 / TMath::Sin(TMath::DegToRad() * ig) / 10.5); // fcorr0->Eval(ig));
      gsigmaCM0->SetPointError(ig, 0, TMath::Sqrt(sigmaCM0[ig]) * scale0 / TMath::Sin(TMath::DegToRad() * ig) / 10.5);

      gsigmaLab1->SetPoint(ig, ig, sigmaLab1[ig]);
      gsigmaLab1->SetPointError(ig, 0, TMath::Sqrt(sigmaLab1[ig]));
      gsigmaCM1->SetPoint(ig, ig, sigmaCM1[ig] * scale1 / TMath::Sin(TMath::DegToRad() * ig) / fcorr1->Eval(ig));
      gsigmaCM1->SetPointError(ig, 0, TMath::Sqrt(sigmaCM1[ig]) * scale1 / TMath::Sin(TMath::DegToRad() * ig));

      gsigmaLab2->SetPoint(ig, ig, sigmaLab2[ig]);
      gsigmaLab2->SetPointError(ig, 0, TMath::Sqrt(sigmaLab2[ig]));
      gsigmaCM2->SetPoint(ig, ig, sigmaCM2[ig] * scale2 / TMath::Sin(TMath::DegToRad() * ig));
      gsigmaCM2->SetPointError(ig, 0, TMath::Sqrt(sigmaCM2[ig]) * scale2 / TMath::Sin(TMath::DegToRad() * ig));

      /*gsigmaLab3->SetPoint(ig, ig, sigmaLab3[ig]);
      gsigmaLab3->SetPointError(ig, 0, TMath::Sqrt(sigmaLab3[ig]));
      gsigmaCM3->SetPoint(ig, ig, sigmaCM3[ig] * scale3 / TMath::Sin(TMath::DegToRad() * ig));
      gsigmaCM3->SetPointError(ig, 0, TMath::Sqrt(sigmaCM3[ig]) * scale3 / TMath::Sin(TMath::DegToRad() * ig));*/

      /*  outputXSFile << ig << " " << sigmaCM0[ig] << " " << sigmaCM1[ig] << "  " << sigmaCM2[ig] << " " <<
         sigmaCM3[ig]
                     << " " << sigmaCM0[ig] * scale0 / TMath::Sin(TMath::DegToRad() * ig) / fcorr0->Eval(ig) << " "
                     << "\n";*/
   }

   TGraphErrors *gsigmaCM2RB = new TGraphErrors();
   gsigmaCM2RB->SetMarkerStyle(22);
   gsigmaCM2RB->SetMarkerSize(1.5);
   gsigmaCM2RB->SetMarkerColor(kBlue);

   auto nPoints = gsigmaCM2->GetN();
   Int_t nPointsRB = 0;

   for (int i = 0; i < nPoints - 3; i += 4) {

      double x1, y1, x2, y2, x3, y3, x4, y4;
      gsigmaCM2->GetPoint(i, x1, y1);
      gsigmaCM2->GetPoint(i + 1, x2, y2);
      gsigmaCM2->GetPoint(i + 2, x3, y3);
      gsigmaCM2->GetPoint(i + 3, x4, y4);

      Double_t x5 = (x4 + x1) / 2.0;
      Double_t y5 = y1 + y2 + y3 + y4;

      gsigmaCM2RB->SetPoint(nPointsRB, x5, y5);

      ++nPointsRB;
   }

   // Merging
   if (kIsMerging)
      m_Chain->Merge("finalDp.root", "C");

   // Adding kinematic lines
   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   //////////////////////////////////////////////////////////////////////////////////////////////////////

   TString fileKine = "./kinematics_a1954/C14pp_gs.txt";
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

   //////////////////////////////////////////////////////////////////////////////////////////////////////

   fileKine = "./kinematics_a1954/C14pp_6.093.txt";
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

   //////////////////////////////////////////////////////////////////////////////////////////////////////

   fileKine = "./kinematics_a1954/C14pp_8.317.txt";
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

   //////////////////////////////////////////////////////////////////////////////////////////////////////

   fileKine = "./kinematics_a1954/C14pp_9.801.txt";
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

   //////////////////////////////////////////////////////////////////////////////////////////////////////

   fileKine = "./kinematics_a1954/C14pp_7.341.txt";
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

   //////////////////////////////////////////////////////////////////////////////////////////////////////

   fileKine = "./kinematics_a1954/C14pp_6.589.txt";
   std::ifstream *kineStr6 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr6->fail()) {
      while (!kineStr6->eof()) {
         *kineStr6 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr6->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec_pp = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   //////////////////////////////////////////////////////////////////////////////////////////////////////

   TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->Draw();
   c1->cd(1);
   Ang_Ener->SetMarkerStyle(20);
   Ang_Ener->SetMarkerSize(0.5);
   Ang_Ener->Draw("col");
   Ang_Ener->GetXaxis()->SetTitle("Angle (deg)");
   Ang_Ener->GetYaxis()->SetTitle("Energy (MeV)");
   Kine_AngRec_EnerRec->SetLineWidth(2);
   Kine_AngRec_EnerRec->SetLineColor(kRed);
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_AngRec_EnerRec_in->SetLineWidth(2);
   Kine_AngRec_EnerRec_in->SetLineColor(kBlue + 3);
   Kine_AngRec_EnerRec_9AMeV->SetLineWidth(2);
   Kine_AngRec_EnerRec_9AMeV->SetLineColor(kRed + 3);
   Kine_AngRec_EnerRec_9AMeV->Draw("SAME");
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp->SetLineWidth(2);
   Kine_AngRec_EnerRec_dp->SetLineColor(kGreen);
   Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp_first->SetLineWidth(2);
   Kine_AngRec_EnerRec_dp_first->SetLineColor(kViolet);
   Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_pp->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_pp->SetLineWidth(2);
   Kine_AngRec_EnerRec_pp->SetLineColor(kGreen + 3);

   c1->cd(2);
   HQval->Draw();
   c1->cd(3);
   Ang_Ener_Xtr->SetMarkerStyle(20);
   Ang_Ener_Xtr->SetMarkerSize(0.5);
   Ang_Ener_Xtr->Draw("col");
   // Kine_AngRec_EnerRec->Draw("SAME");
   // Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   // Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   // Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");
   // Kine_AngRec_EnerRec_9AMeV->Draw("SAME");
   // cutpS->Draw("l");
   c1->Modified();
   c1->cd(4);
   HQval_Xtr->Draw();

   TCanvas *c2 = new TCanvas();
   c2->Divide(2, 2);
   c2->Draw();
   c2->cd(1);
   HQCorr->Draw();
   c2->cd(2);
   QvsZpos->Draw();
   c2->cd(3);
   QcorrvsZpos->Draw();
   c2->cd(4);
   QvsTrackLengthH->Draw();

   TCanvas *c3 = new TCanvas();
   c3->Divide(4, 2);
   c3->Draw();
   c3->cd(1);
   hxpos_fit->Draw();
   c3->cd(2);
   hypos_fit->Draw();
   c3->cd(3);
   hzpos_fit->Draw();
   c3->cd(4);
   x_y_Fit->Draw("zcol");
   c3->cd(5);
   hxpos_fit_Xtr->Draw();
   c3->cd(6);
   hypos_fit_Xtr->Draw();
   c3->cd(7);
   hzpos_fit_Xtr->Draw();
   c3->cd(8);
   x_y_Xtr->Draw("zcol");

   TCanvas *c4 = new TCanvas();
   c4->Divide(2, 3);
   c4->Draw();
   c4->cd(1);
   POCAXtrH->Draw();
   c4->cd(2);
   tracklengthH->Draw();
   c4->cd(3);
   ZposvsRad->Draw();
   c4->cd(4);
   particleQH->Draw();
   c4->cd(5);
   eventMultH->Draw();

   TCanvas *c5 = new TCanvas();
   c5->Divide(3, 3);
   c5->Draw();
   c5->cd(1);
   ICMultH->Draw();
   c5->cd(2);
   QvsAng->Draw();
   c5->cd(3);
   ZposvsAng->Draw();
   c5->cd(4);
   Ang_AngPRA->Draw();
   c5->cd(5);
   zfit_zPRA->Draw();
   c5->cd(6);
   Phi_PhiPRA->Draw();
   c5->cd(7);
   Ang_Phi->Draw();
   c5->cd(8);
   x_Phi->Draw();
   c5->cd(9);
   y_Phi->Draw();

   TCanvas *cCorr = new TCanvas();
   cCorr->Draw();
   hARecvsASca->Draw("zcol");

   TCanvas *c6 = new TCanvas();
   c6->Divide(3, 2);
   c6->Draw();
   c6->cd(1);
   HIC->Draw();
   c6->cd(2);
   ICMultH->Draw();
   c6->cd(3);
   ICTimeH->Draw();
   c6->cd(4);
   ICEvsTime->Draw("zcol");
   c6->cd(5);
   ICQH->Draw();
   c6->cd(6);
   ICAH->Draw();

   TCanvas *cQAng = new TCanvas();
   QvsAng->Draw();

   /*TCanvas *cOrb = new TCanvas();
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
   fOrbLengthvsMomLoss->Draw();*/

   TCanvas *cpid = new TCanvas();
   cpid->Divide(2, 3);
   cpid->Draw("zcol");
   cpid->cd(1);
   ELossvsBrho->Draw("zcol");
   cpid->cd(2);
   ELossvsBrhoZoom->Draw("zcol");
   cpid->cd(3);
   dedxvsBrho->Draw("zcol");
   cpid->cd(4);
   dedxvsBrhoZoom->Draw("zcol");
   cpid->cd(5);
   dedxvsBrhoCond->Draw("zcol");

   TCanvas *c7 = new TCanvas();
   c7->Divide(1, 2);
   c7->Draw();
   c7->cd(1);
   Ang_Ener_PRA->SetMarkerStyle(20);
   Ang_Ener_PRA->SetMarkerSize(0.5);
   Ang_Ener_PRA->Draw("COlZ");
   // Kine_AngRec_EnerRec->Draw("SAME");
   // Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   // Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   // Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");
   c7->cd(2);
   PhiPRAH->Draw();

   TCanvas *c8 = new TCanvas();
   QvsEb->Draw("zcol");
   // QvsMult->Draw("zcol");

   auto leg = new TLegend(0.1, 0.1, 0.2, 0.2);
   leg->AddEntry(gsigmaCM0, "O_1+","lp");
   // leg->AddEntry(gsigmaCM1, "2_1+","lp");
   // leg->AddEntry(gsigmaCM2, "0_2+","lp");
   // leg->AddEntry(gsigmaCM3, "3_1-","lp");
   TCanvas *cxs = new TCanvas("cxs", "cxs", 700, 700);
   // gsigmaCM0->Draw("ALP");
   // gsigmaCM1->Draw("LP");
   gsigmaCM2RB->Draw("ALP");
   // gsigmaCM3->Draw("LP");
   // leg->Draw();

   TCanvas *cgs = new TCanvas("cgs", "cgs", 700, 700);
   gsigmaCM0->Draw("ALP");
   gsigmaCM0->GetXaxis()->SetTitle("#theta (deg)");
   gsigmaCM0->GetYaxis()->SetTitle("E (MeV)");
   gDWBA0->Draw("L");
   /*gAltAnalysis->Draw("LP");*/

   TCanvas *ce1 = new TCanvas();
   hsigmaCM1->Draw("histo");
   // gsigmaCM1->Draw("ALP");
   // gDWBA1->Draw("L");
   // gDWBA2->Draw("L");

   /*auto nPoints = gsigmaCM1->GetN(); // number of points in your TGraph
   for (int i = 0; i < nPoints; ++i) {
      double x, y;
      gsigmaCM1->GetPoint(i, x, y);
      hsigmaCM1->SetBinContent(i, y); // ?
   }*/

   /*TCanvas *ce2 = new TCanvas();
   gsigmaCM2->Draw("ALP");
   gDWBA1->Draw("L");
   gDWBA2->Draw("L");

   TCanvas *ce3 = new TCanvas();
   gsigmaCM3->Draw("ALP");
   gDWBA1->Draw("L");
   gDWBA2->Draw("L");*/

   TCanvas *cKinematics = new TCanvas("cKinematics", "cKinematics", 700, 700);
   Ang_Ener_Xtr->Draw("col");
   Ang_Ener_Xtr->GetYaxis()->SetRangeUser(0, 20);
   Ang_Ener_Xtr->GetXaxis()->SetRangeUser(10, 90);
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_AngRec_EnerRec_9AMeV->Draw("SAME");
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_pp->Draw("ZCOL SAME");

   /*TCanvas *cPID = new TCanvas("cPID", "cPID", 700, 700);
   ELossvsBrhoZoom->Draw("zcol");*/

   // TCanvas *cPRA = new TCanvas("cPRA", "cPRA", 700, 700);
   // Ang_Ener_PRA_Cond->Draw("zcol");

   // TCanvas *cQPRA = new TCanvas("cQPRA", "cQPRA", 700, 700);
   // HQvalPRA->Draw();

   /*TCanvas *cKineBuff = new TCanvas("cKineBuff", "cKineBuff", 1800, 900);
   cKineBuff->Divide(2,1);
   cKineBuff->cd(1);
   Ang_Ener_PRA_Cond->Draw("col");
   Ang_Ener_PRA_Cond->GetXaxis()->SetTitle("Angle (deg)");
   Ang_Ener_PRA_Cond->GetYaxis()->SetTitle("Energy (MeV)");
   //Kine_AngRec_EnerRec->Draw("SAME");
   //Kine_AngRec_EnerRec_9AMeV->Draw("SAME");
   //Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   //Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   //Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");
   cKineBuff->cd(2);
   HQvalPRA->Draw();
   HQvalPRA->GetXaxis()->SetTitle("Excitation Energy (MeV)");
   HQvalPRA->GetYaxis()->SetTitle("Counts");*/

   // TCanvas *cKineFit = new TCanvas("cKineFit", "cKineFit", 1800, 900);
   TCanvas *cKineFit = new TCanvas("cKineFit", "cKineFit", 700, 700);
   // cKineFit->Divide(2, 1);
   cKineFit->Draw();
   cKineFit->cd(1);
   // Ang_Ener->Draw("col");
   // Ang_Ener->GetXaxis()->SetTitle("Angle (deg)");
   // Ang_Ener->GetYaxis()->SetTitle("Energy (MeV)");
   /*Kine_AngRec_EnerRec->Draw("SAME");
   Kine_AngRec_EnerRec_9AMeV->Draw("SAME");
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
   Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");*/

   // cKineFit->cd(2);
   HQval->GetXaxis()->SetTitle("Excitation Energy (MeV)");
   HQval->GetYaxis()->SetTitle("Counts");
   HQval->Draw();

   TCanvas *cTrackLength = new TCanvas("cTrackLength", "cTrackLength", 700, 700);
   QvsTrackLengthH->Draw();

   TF1 *g1 = new TF1("m1", "gaus", -2.0, 2.0);
   TF1 *g2 = new TF1("m2", "gaus", 5.8, 6.2);
   // TF1 *g10 = new TF1("m10", "gaus", 6.0, 6.7);
   TF1 *g3 = new TF1("m3", "gaus", 6.4, 6.9);
   TF1 *g4 = new TF1("m4", "gaus", 6.8, 7.3);
   TF1 *g5 = new TF1("m5", "gaus", 7.3, 7.7);
   TF1 *g6 = new TF1("m6", "gaus", 8.2, 8.6);
   /*TF1 *g7 = new TF1("m7", "gaus", 8.6, 9.6);
   TF1 *g8 = new TF1("m8", "gaus", 9.7, 10.1);
   TF1 *g9 = new TF1("m9", "gaus", 10.1, 11.0);*/

   /*TF1 *total_gaus =
      new TF1("total_gaus", "gaus(0)+gaus(3)+gaus(6)+gaus(9)+gaus(12)+gaus(15)+gaus(18)+gaus(21)+gaus(24)+gaus(27)",
      -2.0, 11.0);  define combined function*/
   TF1 *total_gaus =
      new TF1("total_gaus", "gaus(0)+gaus(3)+gaus(6)+gaus(9)+gaus(12)", -2.0, 8.0); /* define combined function*/

   total_gaus->SetParameter(4, 6.09);
   total_gaus->SetParLimits(4, 5.95, 6.15);
   total_gaus->SetParameter(5, 0.15);
   total_gaus->SetParLimits(5, 0.1, 0.15);

   /*total_gaus->SetParameter(7,6.6);
   total_gaus->SetParLimits(7,6.5,6.65);
   total_gaus->SetParameter(8,0.15);
   total_gaus->SetParLimits(8,0.1,0.2);*/

   total_gaus->SetParameter(7, 6.7);
   total_gaus->SetParLimits(7, 6.6, 6.8);
   total_gaus->SetParameter(8, 0.15);
   total_gaus->SetParLimits(8, 0.15, 0.2);

   total_gaus->SetParameter(10, 7.0);
   total_gaus->SetParLimits(10, 6.9, 7.05);
   total_gaus->SetParameter(11, 0.15);
   total_gaus->SetParLimits(11, 0.15, 0.17);

   total_gaus->SetParameter(13, 7.27);
   total_gaus->SetParLimits(13, 7.25, 7.3);
   total_gaus->SetParameter(14, 0.15);
   total_gaus->SetParLimits(14, 0.14, 0.150);

   HQCorr->Fit(g1, "R");
   HQCorr->Fit(g2, "R+");
   // HQCorr->Fit(g10, "R+");
   HQCorr->Fit(g3, "R+");
   HQCorr->Fit(g4, "R+");
   HQCorr->Fit(g5, "R+");
   /*HQCorr->Fit(g6, "R+");
   HQCorr->Fit(g7, "R+");
   HQCorr->Fit(g8, "R+");
   HQCorr->Fit(g9, "R+");*/
   total_gaus->SetNpx(1000);

   Double_t par[18]; /* parameter array */

   /* get parameters from the fit, it first fits & takes the parameter from there */
   g1->GetParameters(&par[0]);
   g2->GetParameters(&par[3]);
   // g10->GetParameters(&par[6]);
   g3->GetParameters(&par[6]);
   g4->GetParameters(&par[9]);
   g5->GetParameters(&par[12]);
   /*g6->GetParameters(&par[18]);
   g7->GetParameters(&par[21]);
   g8->GetParameters(&par[24]);
   g9->GetParameters(&par[27]);*/

   total_gaus->SetParameters(par);
   // HQval->Fit("total_gaus","","",-2.0,9.0); /*fitting gaussian curve on the histogram */
   HQCorr->Fit("total_gaus", "", "", -2.0, 8.0);
   total_gaus->GetParameters(&par[0]);

   auto gfit1 = new TF1("gfit1", "gaus(0)", -2.0, 2.0);
   gfit1->SetParameter(0, par[0]);
   gfit1->SetParameter(1, par[1]);
   gfit1->SetParameter(2, par[2]);
   gfit1->SetLineColor(kBlue);
   gfit1->SetNpx(1000);
   auto gfit2 = new TF1("gfit2", "gaus(0)", 5.0, 7.0);
   gfit2->SetParameter(0, par[3]);
   gfit2->SetParameter(1, par[4]);
   gfit2->SetParameter(2, par[5]);
   gfit2->SetLineColor(kBlue);
   gfit2->SetNpx(1000);
   auto gfit10 = new TF1("gfit10", "gaus(0)", 5.0, 9.0);
   gfit10->SetParameter(0, par[6]);
   gfit10->SetParameter(1, par[7]);
   gfit10->SetParameter(2, par[8]);
   gfit10->SetLineColor(kBlue);
   gfit10->SetNpx(1000);
   auto gfit3 = new TF1("gfit3", "gaus(0)", 5.0, 9.0);
   gfit3->SetParameter(0, par[9]);
   gfit3->SetParameter(1, par[10]);
   gfit3->SetParameter(2, par[11]);
   gfit3->SetLineColor(kBlue);

   // Number of Counts
   Double_t cnts = gfit3->Integral(5.0, 9.0);
   std::cout << " Number of counts " << cnts / HQCorr->GetBinWidth(1) << "\n";
   std::cout << " Peak " << gfit3->Eval(7.0) << "\n";

   gfit3->SetNpx(1000);
   auto gfit4 = new TF1("gfit4", "gaus(0)", 5.0, 9.0);
   gfit4->SetParameter(0, par[12]);
   gfit4->SetParameter(1, par[13]);
   gfit4->SetParameter(2, par[14]);
   gfit4->SetLineColor(kBlue);
   gfit4->SetNpx(1000);
   auto gfit5 = new TF1("gfit5", "gaus(0)", 5.0, 9.0);
   gfit5->SetParameter(0, par[15]);
   gfit5->SetParameter(1, par[16]);
   gfit5->SetParameter(2, par[17]);
   gfit5->SetLineColor(kBlue);
   gfit5->SetNpx(1000);
   /*auto gfit6 = new TF1("gfit6", "gaus(0)", 5.0, 9.0);
   gfit6->SetParameter(0, par[18]);
   gfit6->SetParameter(1, par[19]);
   gfit6->SetParameter(2, par[20]);
   gfit6->SetLineColor(kBlue);
   gfit6->SetNpx(1000);
   auto gfit7 = new TF1("gfit7", "gaus(0)", 8.0, 10.0);
   gfit7->SetParameter(0, par[21]);
   gfit7->SetParameter(1, par[22]);
   gfit7->SetParameter(2, par[23]);
   gfit7->SetLineColor(kBlue);
   gfit7->SetNpx(1000);
   auto gfit8 = new TF1("gfit8", "gaus(0)", 9.5, 10.5);
   gfit8->SetParameter(0, par[24]);
   gfit8->SetParameter(1, par[25]);
   gfit8->SetParameter(2, par[26]);
   gfit8->SetLineColor(kBlue);
   gfit8->SetNpx(1000);
   auto gfit9 = new TF1("gfit9", "gaus(0)", 10.0, 11.0);
   gfit9->SetParameter(0, par[27]);
   gfit9->SetParameter(1, par[28]);
   gfit9->SetParameter(2, par[29]);
   gfit9->SetLineColor(kBlue);
   gfit9->SetNpx(1000);*/

   TCanvas *cqcorr = new TCanvas("cqcorr", "cqcorr", 700, 700);
   HQCorr->Draw();
   gfit1->Draw("same l");
   gfit2->Draw("same l");
   gfit10->Draw("same l");
   gfit3->Draw("same l");
   gfit4->Draw("same l");
   // gfit5->Draw("same l");
   /*gfit6->Draw("same l");
   gfit7->Draw("same l");
   gfit8->Draw("same l");
   gfit9->Draw("same l");*/
   // total_gaus->Draw("same l");

   // Test of kinematics
   /* TGraphErrors *kineTest = new TGraphErrors();
    Double_t gamma = 1.405;

    for(auto i=0;i<45;++i){

      angle = i;
        Double_t theta_cm = 180.0-(angle+TMath::RadToDeg()*TMath::ASin(gamma * TMath::Sin(angle*TMath::DegToRad())));//;
    high value kineTest->SetPoint(kineTest->GetN(),angle,theta_cm);


      }

     for(auto i=0;i<45;++i){

      angle = i;
        Double_t theta_cm = TMath::RadToDeg()*TMath::ASin(gamma * TMath::Sin(angle*TMath::DegToRad())  )-angle ;//low
    value

        kineTest->SetPoint(kineTest->GetN(),angle,theta_cm);


      }


     TCanvas *cKineTest = new TCanvas("cKineTest", "cKineTest", 700, 700);
     kineTest->Draw("AP");*/

   /*TCanvas *c2 = new TCanvas();
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
   multvsnumpoints->Draw("zcol");
   c6->cd(9);
   eventMultH->Draw();

    */

   /*  TCanvas *c7 = new TCanvas();
   c7->Divide(2, 2);
   c7->Draw();
   c7->cd(1);
   HQval_Xtr_recalc->Draw();
   c7->cd(2);
   HQval_Xtr_recalc_cutps->Draw();
   c7->cd(3);
   QvsEb->Draw("zcol");
   c7->cd(4);
   ZposvsRad->Draw("zcol");*/

   /*TCanvas *cEx = new TCanvas();
   cEx->Divide(2, 5);
   for (auto cIndex = 0; cIndex < 10; ++cIndex) {
      cEx->cd(cIndex + 1);
      // ExZ[cIndex]->Draw();
      HQCorrArray[cIndex]->Draw();
      }*/

   /*TCanvas *cChi = new TCanvas();
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


   TCanvas *c8 = new TCanvas();
   c8->Divide(2, 2);
   c8->Draw();
   c8->cd(1);
   HQCorr->Draw();
   c8->cd(2);
   QcorrvsZpos->Draw();
   c8->cd(3);
   //QvsEvent->Draw();
   c8->cd(4);
   QvsTrackLengthH->Draw();*/
}
