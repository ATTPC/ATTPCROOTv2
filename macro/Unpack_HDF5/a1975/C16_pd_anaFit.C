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

Int_t GetMaxAngleIndex(const std::vector<std::unique_ptr<AtFittedTrack>> &tracks)
{

   // Range of values to exclude
   Double_t lowerBound = 90;
   Double_t upperBound = 180;

   // Lambda function to check if value falls within the range
   auto isInRange = [=](AtFittedTrack &track) {
      auto [energy, energyXtr, theta, phi, energyPRA, thetaPRA, phiPRA] = track.GetEnergyAngles();
      return theta >= lowerBound && theta <= upperBound;
   };

   // Lambda function to compare CustomObjects based on value
   auto compareTracks = [](AtFittedTrack &a, AtFittedTrack &b) {
      auto [energyb, energyXtrb, thetab, phib, energyPRAb, thetaPRAb, phiPRAb] = b.GetEnergyAngles();
      auto [energya, energyXtra, thetaa, phia, energyPRAa, thetaPRAa, phiPRAa] = a.GetEnergyAngles();
      return thetab > thetaa;
   };

   return 0;
}

void C16_pd_anaFit()
{
   FairRunAna *run = new FairRunAna();

   // Output
   Double_t _theta = 0.0, _energy = 0.0, _exEnergy = 0.0, _exEnergyCorr = 0.0, _zVertex = 0.0, _thetacm = 0.0;
   TFile *fileOut = TFile::Open("C15_pd_analysis.root", "RECREATE");
   TTree *treeOut = new TTree("output", "output");
   treeOut->Branch("_theta", &_theta, "_theta/D");
   treeOut->Branch("_energy", &_energy, "_energy/D");
   treeOut->Branch("_exEnergy", &_exEnergy, "_exEnergy/D");
   treeOut->Branch("_exEnergyCorr", &_exEnergyCorr, "_exEnergyCorr/D");
   treeOut->Branch("_zVertex", &_zVertex, "_zVertex/D");
   treeOut->Branch("_thetacm", &_thetacm, "_thetacm/D");

   TH2F *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 179, 1000, 0, 200.0);
   TH2F *Ang_Ener_PRAC = new TH2F("Ang_Ener_PRAC", "Ang_Ener_PRAC", 1000, 0, 100, 1000, 0, 200.0);
   TH2F *ELossvsBrho = new TH2F("ELossvsBrho", "ELossvsBrho", 4000, 0, 25000, 1000, 0, 4);
   TH2F *dedxvsBrho = new TH2F("dedxvsBrho", "dedxvsBrho", 4000, 0, 10000, 1000, 0, 4);
   TH2F *hVxVy = new TH2F("hVxVy", "hVxVy", 1000, 0, 20, 1000, 0, 20);
   TH1F *henergyIC = new TH1F("henergyIC", "henergyIC", 2048, 0, 2047);
   TH2F *QvsTrackLengthH = new TH2F("QvsTrackLengthH", "QvsTrackLengthH", 1000, -10, 50, 1000, 0, 1000);

   auto *hex = new TH1F("hex", "hex", 600, -5, 55);
   auto *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);
   auto *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 1000, -1000, 1000);
   auto *HQCorr = new TH1F("HQCorr", "HQCorr", 600, -5, 55);
   auto *QcorrvsZpos = new TH2F("QcorrvsZpos", "QcorrvsZpos", 1000, -10, 10, 200, -100, 100);
   auto *QcorrvsTrackLengthH = new TH2F("QcorrvsTrackLengthH", "QcorrvsTrackLengthH", 1000, -10, 50, 1000, 0, 1000);

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
   Double_t m_C15 = 15.0105993 * 931.49401;
   Double_t m_C12 = 12.00 * 931.49401;
   Double_t m_C16 = 16.0147 * 931.49401;
   Double_t m_C17 = 17.0226 * 931.49401;

   Double_t m_a = 4.00260325415 * 931.49401;
   Double_t m_O16 = 15.99491461956 * 931.49401;

   Double_t Ebeam_buff = 163.0; // 192.0;
   Double_t m_b;
   Double_t m_B;

   m_b = m_d;
   m_B = m_C15;

   Double_t Am = 1.0;

   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString dataDir = dir + "/macro/Unpack_HDF5/a1975/C16_pp_data/";

   std::vector<std::pair<TString, TString>> filepairs;
   filepairs.push_back(std::make_pair("run_0106.root", "run_0106_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0107.root", "run_0107_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0108.root", "run_0108_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0109.root", "run_0109_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0110.root", "run_0110_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0111.root", "run_0111_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0112.root", "run_0112_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0113.root", "run_0113_FRIB_sorted.root"));
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

   TCutG *cutp = new TCutG("CUTP", 31);
   cutp->SetVarX("ELossvsBrho");
   cutp->SetVarY("");
   cutp->SetTitle("Graph");
   cutp->SetFillStyle(1000);
   cutp->SetPoint(0, -5.846709, 0.9912258);
   cutp->SetPoint(1, 130.7546, 0.4809879);
   cutp->SetPoint(2, 759.1204, 0.2708899);
   cutp->SetPoint(3, 2179.774, 0.1781194);
   cutp->SetPoint(4, 5567.485, 0.08534887);
   cutp->SetPoint(5, 8381.472, 0.05806342);
   cutp->SetPoint(6, 11905.78, 0.05806342);
   cutp->SetPoint(7, 13927.48, 0.05260633);
   cutp->SetPoint(8, 16850.75, 0.05260633);
   cutp->SetPoint(9, 19009.05, 0.05260633);
   cutp->SetPoint(10, 22342.12, 0.05806342);
   cutp->SetPoint(11, 24281.86, 0.05806342);
   cutp->SetPoint(12, 24609.7, 0.07989178);
   cutp->SetPoint(13, 24336.5, 0.1126343);
   cutp->SetPoint(14, 22779.25, 0.12082);
   cutp->SetPoint(15, 18927.09, 0.12082);
   cutp->SetPoint(16, 14992.97, 0.1399198);
   cutp->SetPoint(17, 11222.78, 0.1535625);
   cutp->SetPoint(18, 8080.949, 0.186305);
   cutp->SetPoint(19, 4966.44, 0.2572472);
   cutp->SetPoint(20, 2890.1, 0.3254608);
   cutp->SetPoint(21, 1442.127, 0.4346026);
   cutp->SetPoint(22, 1005.003, 0.5273731);
   cutp->SetPoint(23, 704.4799, 0.6065009);
   cutp->SetPoint(24, 622.5192, 0.7592994);
   cutp->SetPoint(25, 595.1989, 0.8629841);
   cutp->SetPoint(26, 349.3166, 0.9830401);
   cutp->SetPoint(27, 212.7153, 1.018511);
   cutp->SetPoint(28, 21.47355, 0.9857687);
   cutp->SetPoint(29, 21.47355, 0.9148265);
   cutp->SetPoint(30, -5.846709, 0.9912258);

   TCutG *cutd = new TCutG("CUTD", 19);
   cutd->SetVarX("ELossvsBrho");
   cutd->SetVarY("");
   cutd->SetTitle("Graph");
   cutd->SetFillStyle(1000);
   cutd->SetPoint(0, 217.0286, 1.702519);
   cutd->SetPoint(1, 520.3927, 1.360532);
   cutd->SetPoint(2, 657.8545, 1.187317);
   cutd->SetPoint(3, 842.717, 0.9874547);
   cutd->SetPoint(4, 1056.02, 0.8764199);
   cutd->SetPoint(5, 1496.846, 0.7476195);
   cutd->SetPoint(6, 2041.953, 0.6143777);
   cutd->SetPoint(7, 2525.44, 0.5433155);
   cutd->SetPoint(8, 3226.969, 0.4900188);
   cutd->SetPoint(9, 3629.875, 0.458929);
   cutd->SetPoint(10, 3942.719, 0.3923081);
   cutd->SetPoint(11, 3900.058, 0.3123631);
   cutd->SetPoint(12, 1596.387, 0.4278393);
   cutd->SetPoint(13, 842.717, 0.5521983);
   cutd->SetPoint(14, 477.7321, 0.8364473);
   cutd->SetPoint(15, 174.368, 1.222848);
   cutd->SetPoint(16, 112.7471, 1.595925);
   cutd->SetPoint(17, 207.5484, 1.711402);
   cutd->SetPoint(18, 217.0286, 1.702519);

   // Kinematic cut
   TCutG *cutk = new TCutG("CUTk", 13);
   cutk->SetVarX("Ang_Ener");
   cutk->SetVarY("");
   cutk->SetTitle("Graph");
   cutk->SetFillStyle(1000);
   cutk->SetPoint(0, 22.54659, 192.1175);
   cutk->SetPoint(1, 22.88565, 152.0833);
   cutk->SetPoint(2, 33.34008, 82.12065);
   cutk->SetPoint(3, 33.90519, 58.79975);
   cutk->SetPoint(4, 30.23201, 30.03731);
   cutk->SetPoint(5, 35.54399, 17.5995);
   cutk->SetPoint(6, 41.5341, 45.58458);
   cutk->SetPoint(7, 42.15571, 91.83769);
   cutk->SetPoint(8, 32.83149, 174.2382);
   cutk->SetPoint(9, 29.55388, 196.393);
   cutk->SetPoint(10, 22.37706, 192.1175);
   cutk->SetPoint(11, 22.6031, 190.9515);
   cutk->SetPoint(12, 22.54659, 192.1175);

   // Angular distributions
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

   for (auto iFile : filepairs) {

      // GET Data
      TFile *file = new TFile((dataDir + iFile.first).Data(), "READ");
      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Processing file : " << iFile.first.Data() << "\n";
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader ReaderTracking("cbmsim", file);
      TTreeReaderValue<TClonesArray> trackingArray(ReaderTracking, "AtTrackingEvent");
      TTreeReaderValue<TClonesArray> eventHArray(ReaderTracking, "AtEventH");

      // FRIB data
      TFile *fileFRIB = new TFile((dataDir + iFile.second).Data(), "READ");
      TTree *treeFRIB = (TTree *)fileFRIB->Get("FRIB_output_tree");
      Int_t nEventsFRIB = treeFRIB->GetEntries();
      std::cout << " Number of FRIB DAQ events : " << nEventsFRIB << std::endl;

      TTreeReader Reader2("FRIB_output_tree", fileFRIB);
      TTreeReaderValue<ULong64_t> ts(Reader2, "timestamp");
      TTreeReaderValue<std::vector<Float_t>> energyIC(Reader2, "energy");
      TTreeReaderValue<std::vector<Float_t>> timeIC(Reader2, "time");
      TTreeReaderValue<UInt_t> multIC(Reader2, "mult");
      TTreeReaderValue<std::string> fribEvName(Reader2, "eventName");

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
         if (i % 1000 == 0)
            std::cout << " Event Number : " << i << "\n";

         ReaderTracking.Next();
         Reader2.Next();

         AtTrackingEvent *trackingEvent = (AtTrackingEvent *)trackingArray->At(0);
         AtEvent *event = (AtEvent *)eventHArray->At(0);

         if (trackingEvent) {

            auto &fittedTracks = trackingEvent->GetFittedTracks();
            // std::cout<<" Number of fitted tracks "<<fittedTracks.size()<<"\n";
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

            // Find track with largets angle
            auto itMax = std::max_element(fittedTracks.begin(), fittedTracks.end(), [](const auto &a, const auto &b) {
               auto [energyb, energyXtrb, thetab, phib, energyPRAb, thetaPRAb, phiPRAb] = b.get()->GetEnergyAngles();
               auto [energya, energyXtra, thetaa, phia, energyPRAa, thetaPRAa, phiPRAa] = a.get()->GetEnergyAngles();
               return (thetab > thetaa) && (thetab < 90 && thetaa < 90);
            });

            Int_t maxAIndex = std::distance(fittedTracks.begin(), itMax);

            /*  std::cout<<" Max Angle Index : "<<maxAIndex<<" - Number of fitted tracks "<<fittedTracks.size()<<"\n";

              for (auto index = 0; index < fittedTracks.size(); ++index) {

                    auto [energy, energyXtr, theta, phi, energyPRA, thetaPRA, phiPRA] =
                    fittedTracks.at(index)->GetEnergyAngles();

                    std::cout<<" Energy : "<<energy<<" - Theta : "<<theta<<" - Index : "<<index<<"\n";


              }  */

            _theta = 0.0;
            _energy = 0.0;
            _exEnergy = 0.0;
            _exEnergyCorr = 0.0;
            _zVertex = 0.0;
            _thetacm = 0.0;

            for (auto index = 0; index < fittedTracks.size(); ++index) {

               if (index != maxAIndex)
                  continue;

               auto [energy, energyXtr, theta, phi, energyPRA, thetaPRA, phiPRA] =
                  fittedTracks.at(index)->GetEnergyAngles();
               auto [iniPos, iniPosPRA, iniPosXtr] = fittedTracks.at(index)->GetVertices();
               auto [pValue, chi2, bChi2, ndf, bNdf, fitConverged] = fittedTracks.at(index)->GetStats();
               auto [charge, brho, eLossADC, dEdxADC, pdg, trackPoints] = fittedTracks.at(index)->GetTrackProperties();
               // auto [ICEnergy,ICTime] = fittedTracks.at(0)->GetIonChamber(); //TODO
               auto [exEnergy, exEnergyXtr] = fittedTracks.at(index)->GetExcitationEnergy();
               auto [distanceXtr, trackLength, POCAXtr] = fittedTracks.at(index)->GetDistances();

               // Conditions
               if (!cutd->IsInside(eLossADC, brho))
                  continue;

               if (energy * Am < 2.0 || energy * Am > 16.0)
                  continue;

               auto [ex_energy, theta_cm] =
                  kine_2b(m_C16, m_p, m_b, m_B, Ebeam_buff, theta * TMath::DegToRad(), energy * Am);

               // Excitation energy vs Beam energy
               for (auto iEb = 0; iEb < 300; ++iEb) {
                  auto [_ex_energy, _theta_cm] =
                     kine_2b(m_C16, m_p, m_b, m_B, iEb, theta * TMath::DegToRad(), energy * Am);
                  QvsEb->Fill(_ex_energy, iEb);
               }

               /*if(eLossADC>1500)
                  continue; */

               // if (!cutk->IsInside(theta, energy * Am))
               // continue;

               if (theta > 90.0 || theta < 10.0)
                  continue;

               if (iniPosXtr.Z() > 500.0)
                  continue;

               Double_t p0 = 1.42194;
               Double_t p1 = -0.000192859;
               Double_t mFactor = 1.00;
               Double_t offSet = 0.721;
               Double_t QcorrZ = 0.0;
               QcorrZ = ex_energy - mFactor * p1 * (iniPosXtr.Z() / 10.0) - p0;
               QcorrZ = QcorrZ + offSet;
               HQCorr->Fill(QcorrZ);
               QcorrvsZpos->Fill(QcorrZ, iniPosXtr.Z() / 10.0);
               QcorrvsTrackLengthH->Fill(QcorrZ, trackLength);

               // Histograms
               Ang_Ener->Fill(theta, energy * Am);
               Ang_Ener_PRAC->Fill(thetaPRA, energyPRA * Am);
               ELossvsBrho->Fill(eLossADC, brho);
               dedxvsBrho->Fill(dEdxADC, brho);
               hex->Fill(ex_energy);
               QvsZpos->Fill(ex_energy, iniPosXtr.Z());

               Double_t vx = TMath::Sin(theta * TMath::DegToRad()) * TMath::Sqrt(energy * Am);
               Double_t vy = TMath::Cos(theta * TMath::DegToRad()) * TMath::Sqrt(energy * Am);

               hVxVy->Fill(vx, vy);

               if (QcorrZ > 0.35 && QcorrZ < 1.3) { // 0.7 MeV

                  Int_t index = theta;
                  ++sigmaLab1[index];
                  Int_t indexCM = theta_cm;
                  ++sigmaCM1[indexCM];
               }

               // Tree
               _theta = theta;
               _energy = energy * Am;
               _exEnergy = ex_energy;
               _exEnergyCorr = QcorrZ;
               _zVertex = iniPosXtr.Z();
               _thetacm = theta_cm;

               // std::cout<<" Event Number : "<<i<<" - "<<_theta<<" - "<<_energy<<" - "<<_exEnergy<<" -
               // "<<_exEnergyCorr<<" - "<<_zVertex<<"\n";
               fileOut->cd();
               treeOut->Fill();

            } // tracks

         } // tracking event

      } // events

   } // Files

   fileOut->cd();
   treeOut->Write();

   // Diff xs graph
   Double_t beamIntensity = 59855710.0;
   Double_t scale0 = 1.0 * (2.0 * TMath::Pi()) / (beamIntensity * 3.16E21 * 1E-27);
   Double_t scale1 = 1.0; // 2.0 * 1.0 * (2.0 * TMath::Pi()) / (beamIntensity * 3.16E21 * 1E-27);
   Double_t scale2 = 0.2;
   Double_t scale3 = 0.1;

   for (auto ig = 1; ig < 360; ++ig) {

      /*  gsigmaLab0->SetPoint(ig, ig, sigmaLab0[ig]);
        gsigmaLab0->SetPointError(ig, 0, TMath::Sqrt(sigmaLab0[ig]));
        gsigmaCM0->SetPoint(ig, ig, sigmaCM0[ig] * scale0 / TMath::Sin(TMath::DegToRad() * ig) / fcorr0->Eval(ig));
        gsigmaCM0->SetPointError(ig, 0, TMath::Sqrt(sigmaCM0[ig]) * scale0 / TMath::Sin(TMath::DegToRad() * ig));*/

      gsigmaLab1->SetPoint(ig, ig, sigmaLab1[ig]);
      gsigmaLab1->SetPointError(ig, 0, TMath::Sqrt(sigmaLab1[ig]));
      gsigmaCM1->SetPoint(ig, ig, sigmaCM1[ig] * scale1 / TMath::Sin(TMath::DegToRad() * ig));
      gsigmaCM1->SetPointError(ig, 0, TMath::Sqrt(sigmaCM1[ig]) * scale1 / TMath::Sin(TMath::DegToRad() * ig));

      /*  gsigmaLab2->SetPoint(ig, ig, sigmaLab2[ig]);
        gsigmaLab2->SetPointError(ig, 0, TMath::Sqrt(sigmaLab2[ig]));
        gsigmaCM2->SetPoint(ig, ig, sigmaCM2[ig] * scale2 / TMath::Sin(TMath::DegToRad() * ig));
        gsigmaCM2->SetPointError(ig, 0, TMath::Sqrt(sigmaCM2[ig]) * scale2 / TMath::Sin(TMath::DegToRad() * ig));

        gsigmaLab3->SetPoint(ig, ig, sigmaLab3[ig]);
        gsigmaLab3->SetPointError(ig, 0, TMath::Sqrt(sigmaLab3[ig]));
        gsigmaCM3->SetPoint(ig, ig, sigmaCM3[ig] * scale3 / TMath::Sin(TMath::DegToRad() * ig));
        gsigmaCM3->SetPointError(ig, 0, TMath::Sqrt(sigmaCM3[ig]) * scale3 / TMath::Sin(TMath::DegToRad() * ig));*/

      /*  outputXSFile << ig << " " << sigmaCM0[ig] << " " << sigmaCM1[ig] << "  " << sigmaCM2[ig] << " " <<
         sigmaCM3[ig]
                     << " " << sigmaCM0[ig] * scale0 / TMath::Sin(TMath::DegToRad() * ig) / fcorr0->Eval(ig) << " "
                     << "\n";*/
   }

   TCanvas *cxs = new TCanvas();
   // gsigmaCM0->Draw("ALP");
   gsigmaCM1->Draw("ALP");
   // gsigmaCM2->Draw("LP");
   // gsigmaCM3->Draw("LP");

   TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->Draw();
   c1->cd(1);
   Ang_Ener->SetMarkerStyle(20);
   Ang_Ener->SetMarkerSize(0.5);
   Ang_Ener->Draw("col");
   Ang_Ener->GetXaxis()->SetTitle("Angle (deg)");
   Ang_Ener->GetYaxis()->SetTitle("Energy (MeV)");
   c1->cd(2);
   Ang_Ener_PRAC->Draw("col");
   c1->cd(3);
   hVxVy->Draw("zcol");

   TCanvas *cpid = new TCanvas();
   cpid->Divide(2, 2);
   cpid->Draw("zcol");
   cpid->cd(1);
   ELossvsBrho->Draw("zcol");

   // cutd->Draw("l");
   cpid->cd(2);
   dedxvsBrho->Draw("zcol");

   TCanvas *c_ExEner = new TCanvas();
   c_ExEner->Divide(2, 1);
   c_ExEner->cd(1);
   hex->Draw();
   c_ExEner->cd(2);
   QvsTrackLengthH->Draw("zcol");

   TCanvas *c_ExVsZ = new TCanvas();
   QvsZpos->Draw("zcol");

   TCanvas *c_ExEnerBeam = new TCanvas();
   QvsEb->Draw("zcol");

   TCanvas *c_IC = new TCanvas();
   henergyIC->Draw();

   TCanvas *c_ExEner_corr = new TCanvas();
   c_ExEner_corr->Divide(2, 2);
   c_ExEner_corr->cd(1);
   QcorrvsZpos->Draw("zcol");
   c_ExEner_corr->cd(2);
   HQCorr->Draw();
   c_ExEner_corr->cd(3);
   QcorrvsTrackLengthH->Draw("zcol");
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
