Bool_t compareEventName(std::string &getname, std::string &fribname)
{
   // Parsing FRIB event number
   std::regex fribregex("evt(\\d+)_\\d+");
   std::string result = std::regex_replace(fribname, fribregex, "$1\n");

   int fribnumber;
   std::istringstream iss(result);
   while (iss >> fribnumber) {
      // std::cout << fribnumber << std::endl;
   }

   // Parsing GET event name
   std::regex getregex("evt(\\d+)_data");
   result = std::regex_replace(getname, getregex, "$1\n");

   int getnumber;
   std::istringstream isss(result);
   while (isss >> getnumber) {
      // std::cout << getnumber << std::endl;
   }

   return (fribnumber == getnumber) ? 1 : 0;

   return 0;
}

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

void C15_dd_ana_IC(bool accumulateRuns = false)
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   TFile *histFile;
   TH2F *bro_vs_eloss;
   TH2F *bro_vs_eloss_uncut;
   TH2F *bro_vs_dedx;
   TH2F *angle_vs_energy;
   TH2F *angle_vs_energy_lr;
   TH2F *angle_vs_energy_t;
   TH2F *angle_vs_momentum;

   TH1F *HQval;
   TH1F *HQvalp;
   TH2F *QvsEb;
   TH2F *QvsZpos;
   TH2F *vx_vs_vy;

   TH1F *henergyIC;
   std::vector<TString> *processedFiles;

   if (!accumulateRuns) {
      histFile = new TFile("C15_dd_ana_hist.root", "RECREATE");

      bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 1000, 0, 3);
      bro_vs_eloss_uncut = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 1000, 0, 3);
      bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 4000.0, 1000, 0, 3);
      angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 500, 0, 80.0);
      angle_vs_energy_lr = new TH2F("angle_vs_energy_lr", "angle_vs_energy_lr", 720, 0, 179, 500, 0, 100.0);
      angle_vs_energy_t = new TH2F("angle_vs_energy_t", "angle_vs_energy_t", 720, 0, 179, 500, 0, 80.0);
      angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);

      vx_vs_vy = new TH2F("vx_vs_vy", "vx_vs_vy", 1000, 0, 10, 1000, 0, 10);

      HQval = new TH1F("HQval", "HQval", 600, -5, 55);
      HQvalp = new TH1F("HQvalp", "HQvalp", 600, -5, 55);
      QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);
      QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 200, -100, 100);

      henergyIC = new TH1F("henergyIC", "henergyIC", 2048, 0, 2047);

      processedFiles = new std::vector<TString>();
   } else {
      histFile = new TFile("C15_dd_ana_hist.root", "UPDATE");

      bro_vs_eloss = dynamic_cast<TH2F *>(histFile->Get("bro_vs_eloss"));
      bro_vs_eloss_uncut = dynamic_cast<TH2F *>(histFile->Get("bro_vs_eloss_uncut"));
      bro_vs_dedx = dynamic_cast<TH2F *>(histFile->Get("bro_vs_dedx"));
      angle_vs_energy = dynamic_cast<TH2F *>(histFile->Get("angle_vs_energy"));
      angle_vs_energy_lr = dynamic_cast<TH2F *>(histFile->Get("angle_vs_energy_lr"));
      angle_vs_energy_t = dynamic_cast<TH2F *>(histFile->Get("angle_vs_energy_t"));
      angle_vs_momentum = dynamic_cast<TH2F *>(histFile->Get("angle_vs_momentum"));

      HQval = dynamic_cast<TH1F *>(histFile->Get("HQval"));
      HQvalp = dynamic_cast<TH1F *>(histFile->Get("HQvalp"));
      QvsEb = dynamic_cast<TH2F *>(histFile->Get("QvsEb"));
      QvsZpos = dynamic_cast<TH2F *>(histFile->Get("QvsZpos"));

      henergyIC = dynamic_cast<TH1F *>(histFile->Get("henergyIC"));

      histFile->GetObject("ProcessedFiles", processedFiles);
      if (processedFiles == nullptr)
         std::cerr << "Failed to load good events" << std::endl;
   }

   // Proton
   TCutG *cutg = new TCutG("CUTG", 30);
   cutg->SetVarX("bro_vs_eloss");
   cutg->SetVarY("");
   cutg->SetTitle("Graph");
   cutg->SetFillStyle(1000);
   cutg->SetPoint(0, 106.4023, 0.9184422);
   cutg->SetPoint(1, 836.4629, 0.4752705);
   cutg->SetPoint(2, 1372.358, 0.3396269);
   cutg->SetPoint(3, 4548.899, 0.1967351);
   cutg->SetPoint(4, 8199.202, 0.1242537);
   cutg->SetPoint(5, 10490.35, 0.1149347);
   cutg->SetPoint(6, 12758.2, 0.118041);
   cutg->SetPoint(7, 14350.35, 0.1066511);
   cutg->SetPoint(8, 14645.48, 0.07144589);
   cutg->SetPoint(9, 14342.58, 0.03727611);
   cutg->SetPoint(10, 11934.94, 0.03727611);
   cutg->SetPoint(11, 7484.674, 0.03416977);
   cutg->SetPoint(12, 4906.162, 0.03934701);
   cutg->SetPoint(13, 2545.115, 0.108722);
   cutg->SetPoint(14, 883.0625, 0.1967351);
   cutg->SetPoint(15, 199.6016, 0.2536847);
   cutg->SetPoint(16, 90.86913, 0.3686194);
   cutg->SetPoint(17, 114.1689, 0.6026306);
   cutg->SetPoint(18, 83.10253, 0.8314646);
   cutg->SetPoint(19, 90.86913, 0.9143004);
   cutg->SetPoint(20, 106.4023, 0.9184422);

   // Deuteron

   TCutG *cutd = new TCutG("CUTD", 31);
   cutd->SetVarX("bro_vs_eloss");
   cutd->SetVarY("");
   cutd->SetTitle("Graph");
   cutd->SetFillStyle(1000);
   /*cutd->SetPoint(0,718.147,1.56742);
   cutd->SetPoint(1,823.997,1.34694);
   cutd->SetPoint(2,1108.98,1.10459);
   cutd->SetPoint(3,1605.65,0.885933);
   cutd->SetPoint(4,2647.86,0.645408);
   cutd->SetPoint(5,3413.24,0.506924);
   cutd->SetPoint(6,4211.18,0.417638);
   cutd->SetPoint(7,5587.22,0.397595);
   cutd->SetPoint(8,7093.54,0.38484);
   cutd->SetPoint(9,10277.2,0.397595);
   cutd->SetPoint(10,10285.3,0.213557);
   cutd->SetPoint(11,8664.99,0.168003);
   cutd->SetPoint(12,5139.4,0.186224);
   cutd->SetPoint(13,3804.06,0.213557);
   cutd->SetPoint(14,2485.02,0.313775);
   cutd->SetPoint(15,1393.96,0.470481);
   cutd->SetPoint(16,734.432,0.712828);
   cutd->SetPoint(17,465.737,0.949708);
   cutd->SetPoint(18,180.758,1.51822);
   cutd->SetPoint(19,237.754,2.92675);
   cutd->SetPoint(20,758.859,2.91582);
   cutd->SetPoint(21,734.432,1.56195);
   cutd->SetPoint(22,718.147,1.56742);*/
   cutd->SetPoint(0, 57.15925, 2.926749);
   cutd->SetPoint(1, 168.0656, 1.108236);
   cutd->SetPoint(2, 406.9409, 0.8513119);
   cutd->SetPoint(3, 748.1912, 0.5524781);
   cutd->SetPoint(4, 1345.379, 0.4103498);
   cutd->SetPoint(5, 2420.318, 0.2609329);
   cutd->SetPoint(6, 3375.819, 0.180758);
   cutd->SetPoint(7, 4382.507, 0.1151603);
   cutd->SetPoint(8, 5312.415, 0.1169825);
   cutd->SetPoint(9, 7888.855, 0.1206268);
   cutd->SetPoint(10, 9518.325, 0.1151603);
   cutd->SetPoint(11, 10081.39, 0.1479592);
   cutd->SetPoint(12, 10141.11, 0.2080904);
   cutd->SetPoint(13, 9467.138, 0.2226676);
   cutd->SetPoint(14, 7598.792, 0.2372449);
   cutd->SetPoint(15, 6097.29, 0.2463557);
   cutd->SetPoint(16, 5363.602, 0.2955539);
   cutd->SetPoint(17, 3990.069, 0.3629737);
   cutd->SetPoint(18, 2795.693, 0.4540816);
   cutd->SetPoint(19, 1516.004, 0.6526968);
   cutd->SetPoint(20, 842.035, 1.066327);
   cutd->SetPoint(21, 611.6911, 1.308673);
   cutd->SetPoint(22, 577.566, 1.731414);
   cutd->SetPoint(23, 483.7222, 2.135933);
   cutd->SetPoint(24, 517.8472, 2.389213);
   cutd->SetPoint(25, 466.6597, 2.63156);
   cutd->SetPoint(26, 415.4721, 2.883018);
   cutd->SetPoint(27, 313.097, 2.91035);
   cutd->SetPoint(28, 91.28428, 2.919461);
   cutd->SetPoint(29, 57.15925, 2.926749);

   // Deuteron gate on dedx
   TCutG *cutd2 = new TCutG("CUTG", 19);
   cutd2->SetVarX("PID (p/d)");
   cutd2->SetVarY("");
   cutd2->SetTitle("Graph");
   cutd2->SetFillStyle(1000);
   cutd2->SetPoint(0, 42.5536, 1.00769);
   cutd2->SetPoint(1, 54.9278, 0.861278);
   cutd2->SetPoint(2, 111.987, 0.642867);
   cutd2->SetPoint(3, 155.297, 0.51806);
   cutd2->SetPoint(4, 247.416, 0.458057);
   cutd2->SetPoint(5, 430.279, 0.438856);
   cutd2->SetPoint(6, 542.335, 0.414855);
   cutd2->SetPoint(7, 672.264, 0.410055);
   cutd2->SetPoint(8, 595.269, 0.50126);
   cutd2->SetPoint(9, 458.465, 0.618866);
   cutd2->SetPoint(10, 291.413, 0.801275);
   cutd2->SetPoint(11, 192.419, 1.04849);
   cutd2->SetPoint(12, 122.299, 1.39651);
   cutd2->SetPoint(13, 83.801, 1.78533);
   cutd2->SetPoint(14, 65.2397, 2.16695);
   cutd2->SetPoint(15, 54.2403, 2.39016);
   cutd2->SetPoint(16, 12.3055, 2.38056);
   cutd2->SetPoint(17, 35.679, 1.09409);
   cutd2->SetPoint(18, 42.5536, 1.00769);

   // triton
   TCutG *cutt = new TCutG("CUTT", 34);
   cutt->SetVarX("bro_vs_eloss");
   cutt->SetVarY("");
   cutt->SetTitle("Graph");
   cutt->SetFillStyle(1000);
   cutt->SetPoint(0, 214.5209, 2.468662);
   cutt->SetPoint(1, 500.8176, 2.405282);
   cutt->SetPoint(2, 912.1172, 1.114437);
   cutt->SetPoint(3, 1702.457, 0.7362676);
   cutt->SetPoint(4, 2021.013, 0.6285211);
   cutt->SetPoint(5, 2077.466, 0.4954225);
   cutt->SetPoint(6, 1710.522, 0.4489436);
   cutt->SetPoint(7, 1202.446, 0.4933098);
   cutt->SetPoint(8, 476.6235, 0.8038732);
   cutt->SetPoint(9, 113.7122, 1.357394);
   cutt->SetPoint(10, 117.7445, 1.790493);
   cutt->SetPoint(11, 117.7445, 2.29331);
   cutt->SetPoint(12, 206.4562, 2.468662);
   cutt->SetPoint(13, 214.5209, 2.468662);

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
   Double_t m_C16 = 16.0147 * 931.49401;
   Double_t m_C17 = 17.0226 * 931.49401;
   Double_t m_C15 = 15.0106 * 931.49401;

   Double_t m_a = 4.00260325415 * 931.49401;
   Double_t m_O16 = 15.99491461956 * 931.49401;

   Double_t Ebeam_buff = 192.0;
   Double_t m_b;
   Double_t m_B;

   m_b = m_d;
   m_B = m_C15;

   TString FileName = "run_0006.root";
   // std::cout << " Opening File : " << FileName.Data() << std::endl; =
   // dynamic_cast<std::vector<TString>*>(histFile->Get("ProceesedFiles"));

   TString dir = "/home/yassid/fair_install/data/a1954/";

   std::vector<std::pair<TString, TString>> filepairs;
   filepairs.push_back(std::make_pair("run_0014.root", "run_0014_FRIB_sorted.root"));
   /*
   filepairs.push_back(std::make_pair("run_0015.root", "run_0015_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0016.root", "run_0016_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0017.root", "run_0017_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0018.root", "run_0018_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0019.root", "run_0019_FRIB_sorted.root"));
  // filepairs.push_back(std::make_pair("run_0020.root", "run_0020_FRIB_sorted.root"));
  // filepairs.push_back(std::make_pair("run_0021.root", "run_0021_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0022.root", "run_0022_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0023.root", "run_0023_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0024.root", "run_0024_FRIB_sorted.root"));
   //filepairs.push_back(std::make_pair("run_0026.root", "run_0026_FRIB_sorted.root")); // files 25 and 26 it seems like
  GET and NSCL run numbers got messed up? same with 30 and 31
   //filepairs.push_back(std::make_pair("run_0027.root", "run_0027_FRIB_sorted.root"));
   //filepairs.push_back(std::make_pair("run_0028.root", "run_0028_FRIB_sorted.root"));
  // filepairs.push_back(std::make_pair("run_0029.root", "run_0029_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0032.root", "run_0032_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0033.root", "run_0033_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0034.root", "run_0034_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0035.root", "run_0035_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0036.root", "run_0036_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0037.root", "run_0037_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0038.root", "run_0038_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0039.root", "run_0039_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0040.root", "run_0040_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0042.root", "run_0042_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0043.root", "run_0043_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0044.root", "run_0044_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0049.root", "run_0049_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0050.root", "run_0050_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0051.root", "run_0051_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0052.root", "run_0052_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0053.root", "run_0053_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0054.root", "run_0054_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0055.root", "run_0055_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0056.root", "run_0056_FRIB_sorted.root"));
   //filepairs.push_back(std::make_pair("run_0057.root", "run_0057_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0058.root", "run_0058_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0059.root", "run_0059_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0060.root", "run_0060_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0061.root", "run_0061_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0062.root", "run_0062_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0063.root", "run_0063_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0064.root", "run_0064_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0065.root", "run_0065_FRIB_sorted.root"));
   */

   // Final merging
   Bool_t kIsMerging = 0;
   TChain *tpcChain = new TChain("cbmsim");
   TChain *fribChain = new TChain("FRIB_output_tree");

   for (auto iFile : filepairs) {

      // GET Data
      TFile *file = new TFile(iFile.first.Data(), "READ");
      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Processing file : " << iFile.first.Data() << "\n";
      std::cout << " Number of events : " << nEvents << std::endl;
      if (std::find(processedFiles->begin(), processedFiles->end(), iFile.first) != processedFiles->end() &&
          accumulateRuns) {
         std::cout << "Skipping file: " << iFile.first.Data() << "\n";
         continue;
      }
      processedFiles->push_back(iFile.first);

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
      TTreeReaderValue<TClonesArray> eventHArray(Reader1, "AtEventH");

      // FRIB data
      TFile *fileFRIB = new TFile(iFile.second.Data(), "READ");
      TTree *treeFRIB = (TTree *)fileFRIB->Get("FRIB_output_tree");
      Int_t nEventsFRIB = treeFRIB->GetEntries();
      std::cout << " Number of FRIB DAQ events : " << nEventsFRIB << std::endl;

      TTreeReader Reader2("FRIB_output_tree", fileFRIB);
      TTreeReaderValue<ULong64_t> ts(Reader2, "timestamp");
      TTreeReaderValue<std::vector<Float_t>> energyIC(Reader2, "energy");
      TTreeReaderValue<std::vector<Float_t>> timeIC(Reader2, "time");
      TTreeReaderValue<UInt_t> multIC(Reader2, "mult");
      TTreeReaderValue<std::string> fribEvName(Reader2, "eventName");

      if (kIsMerging) {
         tpcChain->Add(iFile.first.Data());
         fribChain->Add(iFile.second.Data());
      }

      if (nEvents != nEventsFRIB + 1) {
         std::cerr << " Error, incompatible number of events! Exiting... "
                   << "\n";
         // std::exit(0);
      }

      ULong64_t fribTSRef = 0;
      ULong64_t getTSRef = 0;

      ULong64_t fribDTS = 0;
      ULong64_t getDTS = 0;

      for (Int_t i = 0; i < nEvents; i++) {

         // eventArray->Clear();
         if (i % 10000 == 0)
            std::cout << " Event Number : " << i << "\n";

         Reader1.Next();
         Reader2.Next();

         AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);
         AtEvent *event = (AtEvent *)eventHArray->At(0);

         if (patternEvent && event) {
            std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
            auto eventName = event->GetEventName();
            auto getTS = event->GetTimestamp(1); // NB New merger only has the internal timestamp

            if (i == 0) {
               fribDTS = 0;
               getDTS = 0;
            } else {

               fribDTS = *ts - fribTSRef;
               getDTS = getTS - getTSRef;
            }

            if ((fribDTS > (getDTS + 5) || fribDTS < (getDTS - 5)) && i != 0) {
               std::cout << " -------------------------------- "
                         << "\n";
               std::cerr << i << "  " << fribDTS << "  " << getDTS << "\n";
               std::cerr << i << "  " << *ts << "  " << getTS << "\n";
               // std::exit(0);
            }

            getTSRef = getTS;
            fribTSRef = *ts;

            // 900 - 1300
            Bool_t goodBeam = false;
            for (auto ener : *energyIC) {
               if (ener > 900 && ener < 1200) {
                  goodBeam = true;
                  henergyIC->Fill(ener);
               }
            }
            if (!goodBeam)
               continue;

            if (*multIC != 1)
               continue;

            /*std::string str2 = "evt2_data";

            if(!compareEventName(eventName,*fribEvName)){
              std::cerr<< " Error, Mismatching event names! Exiting... "<<"\n";
              //if(eventName.compare(str2) == 0)
               std::cout<<eventName <<" "<<*fribEvName<<"\n";
               std::exit(0);
             }*/

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

               // if (theta * TMath::RadToDeg() > 90.0)
               //  continue;

               Double_t B_f = 2.85;

               double bro = B_f * rad / TMath::Sin(theta) / 1000.0;
               double ener = 0;
               Double_t Am = 2.0;

               GetEnergy(Am, 1.0, bro, ener);

               // if (ener * Am < 5.0)
               //  continue;

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
               // if (zpos < 200.0 || zpos > 900)
               // continue;

               if (theta * TMath::RadToDeg() > 90.0)
                  continue;

               // if(bro<1.2)
               // continue;

               // if(eloss>1000)
               // continue;

               Double_t vx = TMath::Sin(theta) * TMath::Sqrt(ener * Am);
               Double_t vy = TMath::Cos(theta) * TMath::Sqrt(ener * Am);

               vx_vs_vy->Fill(vx, vy);

               // if (cutp->IsInside(eloss, bro)) {
               // if (theta * TMath::RadToDeg() > 100.0) {
               angle_vs_energy_t->Fill(theta * TMath::RadToDeg(), ener * Am);
               auto [ex_energy_exp, theta_cm] = kine_2b(m_C15, m_d, m_b, m_B, Ebeam_buff, theta, ener * Am);
               HQvalp->Fill(ex_energy_exp);
               //}
               //}
               bro_vs_eloss_uncut->Fill(eloss, bro);
               // if (cutd->IsInside(eloss, bro) && cutd2->IsInside(dedx, bro)) { // Selection of d
               // if(cutd->IsInside(eloss, bro)){
               if (true) {

                  angle_vs_energy->Fill(theta * TMath::RadToDeg(), ener * Am);
                  auto [ex_energy_exp, theta_cm] = kine_2b(m_C15, m_d, m_b, m_B, Ebeam_buff, theta, ener * Am);

                  HQval->Fill(ex_energy_exp);

                  // Excitation energy vs Beam energy
                  for (auto iEb = 0; iEb < 300; ++iEb) {
                     auto [Qdep, theta_cm_qdep] = kine_2b(m_C15, m_d, m_b, m_B, iEb, theta, ener * Am);
                     QvsEb->Fill(Qdep, iEb);
                     //  }

                     // Rough vertex
                     QvsZpos->Fill(ex_energy_exp, zpos / 10.0);

                  } // deuterons

                  bro_vs_eloss->Fill(eloss, bro);
                  bro_vs_dedx->Fill(dedx, bro);

                  angle_vs_energy_lr->Fill(theta * TMath::RadToDeg(), ener * Am);
               } // cut
            }
         }

      } // nEvents

      file->Close();

   } // Files

   if (kIsMerging) {
      tpcChain->Merge("C16_pp_merged.root", "C");
      fribChain->Merge("C16_pp_merged_FRIB.root", "C");
   }

   std::cout << "Writing histograms" << std::endl;

   histFile->cd();
   histFile->WriteObject(processedFiles, "ProcessedFiles");
   histFile->Write();
   /*
   bro_vs_eloss->Write();
   bro_vs_dedx ->Write();
   angle_vs_energy ->Write();
   angle_vs_energy_lr  ->Write();
   angle_vs_energy_t  ->Write();
   angle_vs_momentum  ->Write();

   HQval  ->Write();
   HQvalp  ->Write();
   QvsEb  ->Write();
   QvsZpos->Write();
   */

   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "C16_pp_gs.txt";
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

   /*************************************************/

   TString fileKine2 = "C16_pp_1.766.txt";
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

   /*************************************************/

   TString fileKine3 = "C16_pp_3.027.txt";
   std::ifstream *kineStr3 = new std::ifstream(fileKine3.Data());
   numKin = 0;

   if (!kineStr3->fail()) {
      while (!kineStr3->eof()) {
         *kineStr3 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr3->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine3 = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   /****************************/

   TCanvas *cvxvy = new TCanvas();
   vx_vs_vy->Draw("zcol");

   TCanvas *c_kn_el_lr = new TCanvas();
   angle_vs_energy_lr->Draw("ZCOL");
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");

   TCanvas *c8 = new TCanvas();
   QvsEb->Draw("zcol");

   TCanvas *cQZ = new TCanvas();
   QvsZpos->Draw();

   TCanvas *c_kn_el = new TCanvas();
   c_kn_el->Divide(2, 1);
   c_kn_el->cd(1);
   angle_vs_energy->Draw("colz");
   Kine_AngRec_EnerRec->SetLineColor(kRed);
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_1m1->Draw("SAME");
   Kine3->SetLineColor(kBlue);
   Kine3->Draw("SAME");
   c_kn_el->cd(2);
   angle_vs_energy_t->Draw("colz");

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

   angle_vs_energy_t->GetXaxis()->SetTitle("#theta (deg)");
   angle_vs_energy_t->GetYaxis()->SetTitle("E (MeV)");
   angle_vs_energy_t->SetTitle("gate on p");

   bro_vs_dedx->GetXaxis()->SetTitle("Brho ");
   bro_vs_dedx->GetYaxis()->SetTitle("dE/dx (au)");
   bro_vs_dedx->SetTitle("PID (p/d)");

   angle_vs_energy->SetStats(0);
   angle_vs_energy_t->SetStats(0);
   bro_vs_dedx->SetStats(0);

   TCanvas *c_ExEner = new TCanvas();
   c_ExEner->Divide(2, 1);
   c_ExEner->cd(1);
   HQval->Draw();
   c_ExEner->cd(2);
   HQvalp->Draw();

   TCanvas *c_IC = new TCanvas();
   henergyIC->Draw();

   // histFile->Close();
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
