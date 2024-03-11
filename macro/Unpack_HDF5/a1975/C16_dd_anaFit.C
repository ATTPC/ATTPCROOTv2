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

void C16_dd_anaFit()
{
   FairRunAna *run = new FairRunAna();

   TH2F *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 179, 1000, 0, 200.0);
   TH2F *Ang_Ener_PRAC = new TH2F("Ang_Ener_PRAC", "Ang_Ener_PRAC", 1000, 0, 100, 1000, 0, 200.0);
   TH2F *ELossvsBrho = new TH2F("ELossvsBrho", "ELossvsBrho", 4000, 0, 25000, 1000, 0, 4);
   TH2F *dedxvsBrho = new TH2F("dedxvsBrho", "dedxvsBrho", 4000, 0, 4000000, 1000, 0, 4);
   TH2F *hVxVy = new TH2F("hVxVy", "hVxVy", 1000, 0, 10, 1000, 0, 10);
   TH1F *henergyIC = new TH1F("henergyIC", "henergyIC", 2048, 0, 2047);
   auto *hex = new TH1F("hex", "hex", 600, -5, 55);
   auto *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);

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

   Double_t Ebeam_buff = 154.0; // 192.0;
   Double_t m_b;
   Double_t m_B;

   m_b = m_d;
   m_B = m_C16;

   Double_t Am = 0.5;

   TString dir = gSystem->Getenv("VMCWORKDIR");
   TString dataDir = dir + "/macro/Unpack_HDF5/a1975/C16_dd_data/";

   std::vector<std::pair<TString, TString>> filepairs;
   filepairs.push_back(std::make_pair("run_0011.root", "run_0011_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0013.root", "run_0013_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0014.root", "run_0014_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0015.root", "run_0015_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0016.root", "run_0016_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0017.root", "run_0017_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0018.root", "run_0018_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0019.root", "run_0019_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0020.root", "run_0020_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0021.root", "run_0021_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0022.root", "run_0022_FRIB_sorted.root"));
   // filepairs.push_back(std::make_pair("run_0023.root", "run_0023_FRIB_sorted.root"));//Weird number of events fro
   // FRIB DAQ filepairs.push_back(std::make_pair("run_0024.root", "run_0024_FRIB_sorted.root"));
   // filepairs.push_back(std::make_pair("run_0025.root", "run_0025_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0026.root", "run_0026_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0027.root", "run_0027_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0028.root", "run_0028_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0029.root", "run_0029_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0030.root", "run_0030_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0031.root", "run_0031_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0032.root", "run_0032_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0033.root", "run_0033_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0034.root", "run_0034_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0035.root", "run_0035_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0036.root", "run_0036_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0037.root", "run_0037_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0038.root", "run_0038_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0039.root", "run_0039_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0040.root", "run_0040_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0041.root", "run_0041_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0042.root", "run_0042_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0043.root", "run_0043_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0044.root", "run_0044_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0045.root", "run_0045_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0046.root", "run_0046_FRIB_sorted.root"));
   // filepairs.push_back(std::make_pair("run_0047.root", "run_0047_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0048.root", "run_0048_FRIB_sorted.root"));
   /*filepairs.push_back(std::make_pair("run_0049.root", "run_0049_FRIB_sorted.root")); //No data block
   filepairs.push_back(std::make_pair("run_0050.root", "run_0050_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0051.root", "run_0051_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0052.root", "run_0052_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0053.root", "run_0053_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0054.root", "run_0054_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0055.root", "run_0055_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0056.root", "run_0056_FRIB_sorted.root"));*/
   filepairs.push_back(std::make_pair("run_0057.root", "run_0057_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0058.root", "run_0058_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0059.root", "run_0059_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0060.root", "run_0060_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0061.root", "run_0061_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0062.root", "run_0062_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0063.root", "run_0063_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0064.root", "run_0064_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0065.root", "run_0065_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0066.root", "run_0066_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0067.root", "run_0067_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0068.root", "run_0068_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0069.root", "run_0069_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0070.root", "run_0070_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0071.root", "run_0071_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0072.root", "run_0072_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0073.root", "run_0073_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0074.root", "run_0074_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0075.root", "run_0075_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0076.root", "run_0076_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0077.root", "run_0077_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0078.root", "run_0078_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0079.root", "run_0079_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0080.root", "run_0080_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0081.root", "run_0081_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0082.root", "run_0082_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0083.root", "run_0083_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0084.root", "run_0084_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0085.root", "run_0085_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0086.root", "run_0086_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0087.root", "run_0087_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0088.root", "run_0088_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0089.root", "run_0089_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0090.root", "run_0090_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0091.root", "run_0091_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0092.root", "run_0092_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0093.root", "run_0093_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0094.root", "run_0094_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0095.root", "run_0095_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0096.root", "run_0096_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0097.root", "run_0097_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0098.root", "run_0098_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0099.root", "run_0099_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0100.root", "run_0100_FRIB_sorted.root"));

   TCutG *cutt = new TCutG("CUTT", 25);
   cutt->SetVarX("ELossvsBrho");
   cutt->SetVarY("");
   cutt->SetTitle("Graph");
   cutt->SetFillStyle(1000);
   cutt->SetPoint(0, 165.5517, 2.378476);
   cutt->SetPoint(1, 188.9105, 1.8812);
   cutt->SetPoint(2, 305.7046, 1.523348);
   cutt->SetPoint(3, 461.4301, 1.249149);
   cutt->SetPoint(4, 640.5143, 1.002835);
   cutt->SetPoint(5, 887.0796, 0.8448221);
   cutt->SetPoint(6, 1201.126, 0.7379311);
   cutt->SetPoint(7, 1551.508, 0.6217452);
   cutt->SetPoint(8, 1813.646, 0.5892131);
   cutt->SetPoint(9, 2096.547, 0.5706234);
   cutt->SetPoint(10, 2158.838, 0.5706234);
   cutt->SetPoint(11, 2184.792, 0.6589247);
   cutt->SetPoint(12, 2153.647, 0.7100465);
   cutt->SetPoint(13, 1517.768, 0.8355272);
   cutt->SetPoint(14, 1089.523, 0.9424183);
   cutt->SetPoint(15, 876.6979, 1.086489);
   cutt->SetPoint(16, 656.0869, 1.272386);
   cutt->SetPoint(17, 549.6745, 1.583764);
   cutt->SetPoint(18, 422.4987, 2.006681);
   cutt->SetPoint(19, 329.0634, 2.332002);
   cutt->SetPoint(20, 271.9641, 2.582963);
   cutt->SetPoint(21, 227.8419, 2.596905);
   cutt->SetPoint(22, 162.9563, 2.564373);
   cutt->SetPoint(23, 165.5517, 2.378476);
   cutt->SetPoint(24, 165.5517, 2.378476);

   TCutG *cutd = new TCutG("CUTD", 20);
   cutd->SetVarX("ELossvsBrho");
   cutd->SetVarY("");
   cutd->SetTitle("Graph");
   cutd->SetFillStyle(1000);
   cutd->SetPoint(0, 48.7938, 1.961132);
   cutd->SetPoint(1, 923.042, 0.9194652);
   cutd->SetPoint(2, 1360.166, 0.7639925);
   cutd->SetPoint(3, 3737.028, 0.4841418);
   cutd->SetPoint(4, 8053.629, 0.3364428);
   cutd->SetPoint(5, 16905.39, 0.289801);
   cutd->SetPoint(6, 22888.53, 0.2664801);
   cutd->SetPoint(7, 24309.18, 0.2198383);
   cutd->SetPoint(8, 24746.3, 0.1576492);
   cutd->SetPoint(9, 24555.06, 0.04881838);
   cutd->SetPoint(10, 10731.01, 0.04104475);
   cutd->SetPoint(11, 8217.55, 0.04881838);
   cutd->SetPoint(12, 6523.694, 0.1343283);
   cutd->SetPoint(13, 3299.904, 0.2276119);
   cutd->SetPoint(14, 1360.166, 0.4375);
   cutd->SetPoint(15, 458.5976, 0.6707089);
   cutd->SetPoint(16, 158.0748, 1.028296);
   cutd->SetPoint(17, -33.16696, 1.510261);
   cutd->SetPoint(18, 48.7938, 1.922264);
   cutd->SetPoint(19, 48.7938, 1.961132);

   // dedx cuts
   auto cuttdedx = new TCutG("CUTTDEDEX", 19);
   cuttdedx->SetVarX("dedxvsBrho");
   cuttdedx->SetVarY("");
   cuttdedx->SetTitle("Graph");
   cuttdedx->SetFillStyle(1000);
   cuttdedx->SetPoint(0, 9474.773, 2.854373);
   cuttdedx->SetPoint(1, 20398.5, 1.965816);
   cuttdedx->SetPoint(2, 37174.23, 1.468925);
   cuttdedx->SetPoint(3, 67604.63, 1.042184);
   cuttdedx->SetPoint(4, 83600.09, 0.9077313);
   cuttdedx->SetPoint(5, 118321.9, 0.8375821);
   cuttdedx->SetPoint(6, 130416.1, 0.8025075);
   cuttdedx->SetPoint(7, 130025.9, 0.6212885);
   cuttdedx->SetPoint(8, 93743.55, 0.685592);
   cuttdedx->SetPoint(9, 54340.1, 0.8317363);
   cuttdedx->SetPoint(10, 21568.9, 1.270169);
   cuttdedx->SetPoint(11, 11425.44, 1.585841);
   cuttdedx->SetPoint(12, 2842.508, 2.100269);
   cuttdedx->SetPoint(13, 1281.975, 2.345791);
   cuttdedx->SetPoint(14, 2062.242, 2.813453);
   cuttdedx->SetPoint(15, 2062.242, 2.918677);
   cuttdedx->SetPoint(16, 9084.64, 2.971289);
   cuttdedx->SetPoint(17, 9474.773, 2.866065);
   cuttdedx->SetPoint(18, 9474.773, 2.854373);

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

            // if(*multIC!=1)
            // continue;

            // Find track with largets angle
            auto itMax = std::max_element(fittedTracks.begin(), fittedTracks.end(), [](const auto &a, const auto &b) {
               auto [energyb, energyXtrb, thetab, phib, energyPRAb, thetaPRAb, phiPRAb] = b.get()->GetEnergyAngles();
               auto [energya, energyXtra, thetaa, phia, energyPRAa, thetaPRAa, phiPRAa] = a.get()->GetEnergyAngles();
               return (thetab > thetaa) && (thetab < 90 && thetaa < 90);
            });

            Int_t maxAIndex = std::distance(fittedTracks.begin(), itMax);

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

               // Conditions
               /*if (!cutd->IsInside(eLossADC, brho))
                continue;*/

               if (!cutt->IsInside(eLossADC, brho))
                  continue;

               /*if (cuttdedx->IsInside(dEdxADC, brho))
                  continue;*/

               if (theta > 90.0 || theta < 10.0)
                  continue;

               /* if(eLossADC<600)
                    continue;

                 if(dEdxADC<20000 || dEdxADC>110000)
                    continue; */

               // Histograms
               Ang_Ener->Fill(theta, energy * Am);
               Ang_Ener_PRAC->Fill(thetaPRA, energyPRA * Am);
               ELossvsBrho->Fill(eLossADC, brho);
               dedxvsBrho->Fill(dEdxADC, brho);

               Double_t vx = TMath::Sin(theta * TMath::DegToRad()) * TMath::Sqrt(energy * Am);
               Double_t vy = TMath::Cos(theta * TMath::DegToRad()) * TMath::Sqrt(energy * Am);

               hVxVy->Fill(vx, vy);
            }
         }

      } // events

   } // Files

   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "C16_dd_gs.txt";
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

   TGraph *kine_gs = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->Draw();
   c1->cd(1);
   Ang_Ener->SetMarkerStyle(20);
   Ang_Ener->SetMarkerSize(0.5);
   Ang_Ener->Draw("col");
   Ang_Ener->GetXaxis()->SetTitle("Angle (deg)");
   Ang_Ener->GetYaxis()->SetTitle("Energy (MeV)");
   kine_gs->Draw("same");
   c1->cd(2);
   Ang_Ener_PRAC->Draw("col");
   c1->cd(3);
   hVxVy->Draw("zcol");

   TCanvas *cpid = new TCanvas();
   cpid->Divide(2, 2);
   cpid->Draw("zcol");
   cpid->cd(1);
   ELossvsBrho->Draw("zcol");
   cutt->Draw("l");
   cutd->Draw("l");
   cpid->cd(2);
   dedxvsBrho->Draw("zcol");

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
