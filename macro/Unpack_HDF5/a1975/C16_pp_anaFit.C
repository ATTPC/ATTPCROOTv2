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

void C16_pp_anaFit()
{
   FairRunAna *run = new FairRunAna();

   TH2F *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 179, 1000, 0, 200.0);
   TH2F *Ang_Ener_PRAC = new TH2F("Ang_Ener_PRAC", "Ang_Ener_PRAC", 1000, 0, 100, 1000, 0, 200.0);
   TH2F *ELossvsBrho = new TH2F("ELossvsBrho", "ELossvsBrho", 4000, 0, 25000, 1000, 0, 4);
   TH2F *dedxvsBrho = new TH2F("dedxvsBrho", "dedxvsBrho", 4000, 0, 10000, 1000, 0, 4);
   TH2F *hVxVy = new TH2F("hVxVy", "hVxVy", 1000, 0, 20, 1000, 0, 20);

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

   Double_t Am = 1.0;

   std::vector<TString> files{"run_0106.root"};

   files.push_back("run_0107.root");
   files.push_back("run_0108.root");
   files.push_back("run_0109.root");
   files.push_back("run_0110.root");
   files.push_back("run_0111.root");
   files.push_back("run_0112.root");
   files.push_back("run_0113.root");
   files.push_back("run_0115.root");
   files.push_back("run_0116.root");
   files.push_back("run_0117.root");
   files.push_back("run_0118.root");
   files.push_back("run_0119.root");
   files.push_back("run_0120.root");
   files.push_back("run_0121.root");
   files.push_back("run_0122.root");
   files.push_back("run_0123.root");
   files.push_back("run_0124.root");
   files.push_back("run_0125.root");
   files.push_back("run_0126.root");
   files.push_back("run_0127.root");
   files.push_back("run_0128.root");
   files.push_back("run_0129.root");
   files.push_back("run_0130.root");
   files.push_back("run_0131.root");
   files.push_back("run_0132.root");
   files.push_back("run_0133.root");
   files.push_back("run_0134.root");
   files.push_back("run_0135.root");
   files.push_back("run_0136.root");
   files.push_back("run_0137.root");
   files.push_back("run_0138.root");
   files.push_back("run_0139.root");
   files.push_back("run_0140.root");
   files.push_back("run_0141.root");
   files.push_back("run_0142.root");
   files.push_back("run_0143.root");
   files.push_back("run_0144.root");
   files.push_back("run_0145.root");
   files.push_back("run_0146.root");
   files.push_back("run_0147.root");
   files.push_back("run_0148.root");
   files.push_back("run_0149.root");
   files.push_back("run_0150.root");
   files.push_back("run_0151.root");
   files.push_back("run_0152.root");
   files.push_back("run_0153.root");
   files.push_back("run_0154.root");
   files.push_back("run_0155.root");
   files.push_back("run_0156.root");
   files.push_back("run_0157.root");
   files.push_back("run_0158.root");
   files.push_back("run_0159.root");
   files.push_back("run_0160.root");
   files.push_back("run_0161.root");
   files.push_back("run_0162.root");
   files.push_back("run_0163.root");
   files.push_back("run_0164.root");
   files.push_back("run_0165.root");
   files.push_back("run_0166.root");
   files.push_back("run_0167.root");
   files.push_back("run_0168.root");
   files.push_back("run_0169.root");
   files.push_back("run_0170.root");
   files.push_back("run_0171.root");
   files.push_back("run_0172.root");
   files.push_back("run_0173.root");
   files.push_back("run_0174.root");
   files.push_back("run_0175.root");
   files.push_back("run_0176.root");
   files.push_back("run_0177.root");
   files.push_back("run_0178.root");
   files.push_back("run_0179.root");
   files.push_back("run_0180.root");
   files.push_back("run_0181.root");
   files.push_back("run_0182.root");
   files.push_back("run_0183.root");
   files.push_back("run_0184.root");
   files.push_back("run_0185.root");
   files.push_back("run_0186.root");
   files.push_back("run_0187.root");
   files.push_back("run_0188.root");
   files.push_back("run_0189.root");

   TCutG *cutt = new TCutG("CUTG", 8);
   cutt->SetVarX("ELossvsBrho");
   cutt->SetVarY("");
   cutt->SetTitle("Graph");
   cutt->SetFillStyle(1000);
   cutt->SetPoint(0, 120.8701, 2.414713);
   cutt->SetPoint(1, 217.4199, 1.590646);
   cutt->SetPoint(2, 679.8425, 1.188423);
   cutt->SetPoint(3, 974.5735, 1.463112);
   cutt->SetPoint(4, 690.0057, 2.090972);
   cutt->SetPoint(5, 303.8066, 2.404903);
   cutt->SetPoint(6, 100.5439, 2.395092);
   cutt->SetPoint(7, 120.8701, 2.414713);

   TCutG *cutt2 = new TCutG("CUTT2", 13);
   cutt2->SetVarX("ELossvsBrho");
   cutt2->SetVarY("");
   cutt2->SetTitle("Graph");
   cutt2->SetFillStyle(1000);
   cutt2->SetPoint(0, 232.374, 2.29419);
   cutt2->SetPoint(1, 294.992, 1.514495);
   cutt2->SetPoint(2, 728.1, 1.120061);
   cutt2->SetPoint(3, 1333.408, 0.8357015);
   cutt2->SetPoint(4, 1688.243, 0.8081828);
   cutt2->SetPoint(5, 1876.097, 1.097129);
   cutt2->SetPoint(6, 1891.752, 1.298932);
   cutt2->SetPoint(7, 1563.007, 1.409007);
   cutt2->SetPoint(8, 827.2452, 1.656675);
   cutt2->SetPoint(9, 561.1186, 2.275845);
   cutt2->SetPoint(10, 368.0464, 2.358401);
   cutt2->SetPoint(11, 237.5921, 2.280431);
   cutt2->SetPoint(12, 232.374, 2.29419);
   cutt2->Draw("l");

   TCutG *cutt3 = new TCutG("CUTt3", 32);
   cutt3->SetVarX("ELossvsBrho");
   cutt3->SetVarY("");
   cutt3->SetTitle("Graph");
   cutt3->SetFillStyle(1000);
   cutt3->SetPoint(0, 296.4843, 2.226385);
   cutt3->SetPoint(1, 167.5781, 1.998336);
   cutt3->SetPoint(2, 280.371, 1.431471);
   cutt3->SetPoint(3, 892.6757, 1.099171);
   cutt3->SetPoint(4, 1843.359, 0.7473234);
   cutt3->SetPoint(5, 3986.426, 0.5974625);
   cutt3->SetPoint(6, 6806.25, 0.5583684);
   cutt3->SetPoint(7, 7192.969, 0.5583684);
   cutt3->SetPoint(8, 7257.422, 0.7994489);
   cutt3->SetPoint(9, 7096.289, 0.8971842);
   cutt3->SetPoint(10, 5775, 0.8711215);
   cutt3->SetPoint(11, 4099.219, 0.923247);
   cutt3->SetPoint(12, 2552.344, 1.060076);
   cutt3->SetPoint(13, 2036.719, 1.170843);
   cutt3->SetPoint(14, 1666.113, 1.28161);
   cutt3->SetPoint(15, 1134.375, 1.542237);
   cutt3->SetPoint(16, 924.9023, 1.835443);
   cutt3->SetPoint(17, 828.2226, 2.096071);
   cutt3->SetPoint(18, 650.9765, 2.226385);
   cutt3->SetPoint(19, 312.5976, 2.239416);
   cutt3->SetPoint(20, 296.4843, 2.226385);

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

   for (auto iFile : files) {

      TFile *file = new TFile(iFile.Data(), "READ");
      std::cout << " Processing file : " << iFile.Data() << "\n";

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader ReaderTracking("cbmsim", file);
      TTreeReaderValue<TClonesArray> trackingArray(ReaderTracking, "AtTrackingEvent");
      TTreeReaderValue<TClonesArray> eventHArray(ReaderTracking, "AtEventH");

      for (Int_t i = 0; i < nEvents; i++) {

         // eventArray->Clear();
         if (i % 1000 == 0)
            std::cout << " Event Number : " << i << "\n";

         ReaderTracking.Next();

         AtTrackingEvent *trackingEvent = (AtTrackingEvent *)trackingArray->At(0);

         if (trackingEvent) {

            auto &fittedTracks = trackingEvent->GetFittedTracks();
            // std::cout<<" Number of fitted tracks "<<fittedTracks.size()<<"\n";

            // Find track with largets angle
            auto itMax = std::max_element(fittedTracks.begin(), fittedTracks.end(), [](const auto &a, const auto &b) {
               auto [energyb, energyXtrb, thetab, phib, energyPRAb, thetaPRAb, phiPRAb] = b.get()->GetEnergyAngles();
               auto [energya, energyXtra, thetaa, phia, energyPRAa, thetaPRAa, phiPRAa] = a.get()->GetEnergyAngles();
               return energyb > energya;
            });

            Int_t maxAIndex = std::distance(fittedTracks.begin(), itMax);

            for (auto index = 0; index < fittedTracks.size(); ++index) {

               if (index != maxAIndex)
                  continue;

               auto [energy, energyXtr, theta, phi, energyPRA, thetaPRA, phiPRA] =
                  fittedTracks.at(0)->GetEnergyAngles();
               auto [iniPos, iniPosPRA, iniPosXtr] = fittedTracks.at(0)->GetVertices();
               auto [pValue, chi2, bChi2, ndf, bNdf, fitConverged] = fittedTracks.at(0)->GetStats();
               auto [charge, brho, eLossADC, dEdxADC, pdg, trackPoints] = fittedTracks.at(0)->GetTrackProperties();
               // auto [ICEnergy,ICTime] = fittedTracks.at(0)->GetIonChamber(); //TODO
               auto [exEnergy, exEnergyXtr] = fittedTracks.at(0)->GetExcitationEnergy();

               // Conditions
               if (!cutp->IsInside(eLossADC, brho))
                  continue;

               /*if(eLossADC>1500)
                  continue; */

               // if (!cutk->IsInside(theta, energy * Am))
               // continue;

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
   cutt2->Draw("l");
   // cutd->Draw("l");
   cpid->cd(2);
   dedxvsBrho->Draw("zcol");
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
