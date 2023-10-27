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

void C16_pp_ana_IC()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   TH2F *bro_vs_eloss = new TH2F("bro_vs_eloss", "bro_vs_eloss", 4000, 0, 25000.0, 1000, 0, 3);
   TH2F *bro_vs_dedx = new TH2F("bro_vs_dedx", "bro_vs_dedx", 4000, 0, 4000.0, 1000, 0, 3);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_energy_lr = new TH2F("angle_vs_energy_lr", "angle_vs_energy_lr", 720, 0, 179, 500, 0, 100.0);
   TH2F *angle_vs_energy_t = new TH2F("angle_vs_energy_t", "angle_vs_energy_t", 720, 0, 179, 500, 0, 80.0);
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);

   TH1F *HQval = new TH1F("HQval", "HQval", 600, -5, 55);
   TH1F *HQvalp = new TH1F("HQvalp", "HQvalp", 600, -5, 55);
   TH2F *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);
   TH2F *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 200, -100, 100);

   TH1F *henergyIC = new TH1F("henergyIC", "henergyIC", 2048, 0, 2047);

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
   TCutG *cutp = new TCutG("CUTP", 30);
   cutp->SetVarX("bro_vs_eloss");
   cutp->SetVarY("");
   cutp->SetTitle("Graph");
   cutp->SetFillStyle(1000);
   cutp->SetPoint(0, 4232.583, 0.1645176);
   cutp->SetPoint(1, 5277.039, 0.1217023);
   cutp->SetPoint(2, 6554.724, 0.106717);
   cutp->SetPoint(3, 8106.198, 0.09922434);
   cutp->SetPoint(4, 9302.759, 0.09815395);
   cutp->SetPoint(5, 10509.46, 0.08209824);
   cutp->SetPoint(6, 10468.9, 0.06390175);
   cutp->SetPoint(7, 8197.461, 0.05426832);
   cutp->SetPoint(8, 5895.601, 0.06497214);
   cutp->SetPoint(9, 4415.109, 0.07460557);
   cutp->SetPoint(10, 3188.127, 0.09280205);
   cutp->SetPoint(11, 2204.513, 0.1377581);
   cutp->SetPoint(12, 1231.039, 0.1827141);
   cutp->SetPoint(13, 287.9859, 0.2854707);
   cutp->SetPoint(14, 176.4421, 0.3197229);
   cutp->SetPoint(15, 105.4596, 0.4032126);
   cutp->SetPoint(16, 85.17888, 0.426761);
   cutp->SetPoint(17, 166.3017, 0.4417463);
   cutp->SetPoint(18, 450.2316, 0.4331833);
   cutp->SetPoint(19, 663.179, 0.3957199);
   cutp->SetPoint(20, 815.2843, 0.3507639);
   cutp->SetPoint(21, 1018.091, 0.3154413);
   cutp->SetPoint(22, 1190.477, 0.291893);
   cutp->SetPoint(23, 1606.232, 0.2555);
   cutp->SetPoint(24, 2295.776, 0.2265997);
   cutp->SetPoint(25, 2620.267, 0.2126847);
   cutp->SetPoint(26, 3127.285, 0.1955586);
   cutp->SetPoint(27, 3644.443, 0.1784325);
   cutp->SetPoint(28, 4303.566, 0.1623768);
   cutp->SetPoint(29, 4232.583, 0.1645176);

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

   Double_t m_a = 4.00260325415 * 931.49401;
   Double_t m_O16 = 15.99491461956 * 931.49401;

   Double_t Ebeam_buff = 192.0; // 154.0; // 192.0;
   Double_t m_b;
   Double_t m_B;

   m_b = m_p;
   m_B = m_C16;

   TString FileName = "run_0006.root";
   // std::cout << " Opening File : " << FileName.Data() << std::endl;
   // TFile *file = new TFile(FileName.Data(), "READ");

   TString dir = "/home/yassid/fair_install/data/a1954/";

   std::vector<std::pair<TString, TString>> filepairs;
   filepairs.push_back(std::make_pair("run_0106.root", "run_0106_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0107.root", "run_0107_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0108.root", "run_0108_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0109.root", "run_0109_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0110.root", "run_0110_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0111.root", "run_0111_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0112.root", "run_0112_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0113.root", "run_0113_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0114.root", "run_0114_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0115.root", "run_0115_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0116.root", "run_0116_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0117.root", "run_0117_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0118.root", "run_0118_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0119.root", "run_0119_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0120.root", "run_0120_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0121.root", "run_0121_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0122.root", "run_0122_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0123.root", "run_0123_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0124.root", "run_0124_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0125.root", "run_0125_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0126.root", "run_0126_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0127.root", "run_0127_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0128.root", "run_0128_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0129.root", "run_0129_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0130.root", "run_0130_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0131.root", "run_0131_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0132.root", "run_0132_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0133.root", "run_0133_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0134.root", "run_0134_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0135.root", "run_0135_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0136.root", "run_0136_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0137.root", "run_0137_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0138.root", "run_0138_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0139.root", "run_0139_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0140.root", "run_0140_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0141.root", "run_0141_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0142.root", "run_0142_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0143.root", "run_0143_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0144.root", "run_0144_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0145.root", "run_0145_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0146.root", "run_0146_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0147.root", "run_0147_FRIB_sorted.root"));
   // filepairs.push_back(std::make_pair("run_0148.root", "run_0148_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0149.root", "run_0149_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0150.root", "run_0150_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0151.root", "run_0151_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0152.root", "run_0152_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0153.root", "run_0153_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0154.root", "run_0154_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0155.root", "run_0155_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0156.root", "run_0156_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0157.root", "run_0157_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0158.root", "run_0158_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0159.root", "run_0159_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0160.root", "run_0160_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0161.root", "run_0161_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0162.root", "run_0162_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0163.root", "run_0163_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0164.root", "run_0164_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0165.root", "run_0165_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0166.root", "run_0166_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0167.root", "run_0167_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0168.root", "run_0168_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0169.root", "run_0169_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0170.root", "run_0170_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0171.root", "run_0171_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0172.root", "run_0172_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0173.root", "run_0173_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0174.root", "run_0174_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0175.root", "run_0175_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0176.root", "run_0176_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0177.root", "run_0177_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0178.root", "run_0178_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0179.root", "run_0179_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0180.root", "run_0180_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0181.root", "run_0181_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0182.root", "run_0182_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0183.root", "run_0183_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0184.root", "run_0184_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0185.root", "run_0185_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0186.root", "run_0186_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0187.root", "run_0187_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0188.root", "run_0188_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0189.root", "run_0189_FRIB_sorted.root"));

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
            auto getTS = event->GetTimestamp(1);

            if (i == 0) {
               fribDTS = 0;
               getDTS = 0;
            } else {

               fribDTS = *ts - fribTSRef;
               getDTS = getTS - getTSRef;
            }

            if (fribDTS > (getDTS + 5) && fribDTS < (getDTS - 5)) {
               std::cerr << i << "  " << fribDTS << "  " << getDTS << "\n";
               std::exit(0);
            }

            getTSRef = getTS;
            fribTSRef = *ts;

            // 900 - 1300
            Bool_t goodBeam = false;
            for (auto ener : *energyIC) {
               if (ener > 900 && ener < 1300) {
                  goodBeam = true;
                  henergyIC->Fill(ener);
               }
            }
            if (!goodBeam)
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
               Double_t Am = 1.0;

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

               /*std::cout << " Brho : " << bro << " - Angle : " << theta * TMath::RadToDeg() << " - Radius : " << rad
                         << " - Energy :" << ener * Am << " - dE     :" << eloss << "\n";*/

               // Selection of events
               if (zpos < 200.0 || zpos > 900)
                  continue;

               // if(theta * TMath::RadToDeg()<13.0)
               //  continue;

               // if(ener*Am>8.0)
               //  continue;

               // if (cutp->IsInside(eloss, bro)) {
               if (theta * TMath::RadToDeg() > 100.0) {
                  angle_vs_energy_t->Fill(theta * TMath::RadToDeg(), ener * Am);
                  auto [ex_energy_exp, theta_cm] = kine_2b(m_C16, m_p, m_b, m_B, Ebeam_buff, theta, ener * Am);
                  HQvalp->Fill(ex_energy_exp);
               }
               //}

               if (cutg->IsInside(eloss, bro)) { // Selection of protons

                  angle_vs_energy->Fill(theta * TMath::RadToDeg(), ener * Am);
                  auto [ex_energy_exp, theta_cm] = kine_2b(m_C16, m_p, m_b, m_B, Ebeam_buff, theta, ener * Am);

                  HQval->Fill(ex_energy_exp);

                  // Excitation energy vs Beam energy
                  for (auto iEb = 0; iEb < 300; ++iEb) {
                     auto [Qdep, theta_cm_qdep] = kine_2b(m_C16, m_p, m_b, m_B, iEb, theta, ener * Am);
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
   TCanvas *c_PID_dedx = new TCanvas();
   c_PID_eloss->cd();
   bro_vs_eloss->Draw("colz");
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
