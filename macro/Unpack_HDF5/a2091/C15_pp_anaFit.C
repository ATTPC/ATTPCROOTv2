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

void C15_pp_anaFit()
{
   FairRunAna *run = new FairRunAna();

   auto *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 179, 1000, 0, 200.0);
   auto *Ang_Ener_PRAC = new TH2F("Ang_Ener_PRAC", "Ang_Ener_PRAC", 1000, 0, 100, 1000, 0, 200.0);
   auto *ELossvsBrho = new TH2F("ELossvsBrho", "ELossvsBrho", 4000, 0, 25000, 1000, 0, 4);
   auto *dedxvsBrho = new TH2F("dedxvsBrho", "dedxvsBrho", 4000, 0, 4000000, 1000, 0, 4);
   auto *hVxVy = new TH2F("hVxVy", "hVxVy", 1000, 0, 10, 1000, 0, 10);
   auto henergyIC = new TH1F("henergyIC", "henergyIC", 2048, 0, 2047);
   auto *hex = new TH1F("hex", "hex", 600, -5, 55);
   auto *QvsEb = new TH2F("QvsEb", "QvsEb", 1000, -5, 15, 300, 0, 300);
   auto *QvsZpos = new TH2F("QvsZpos", "QvsZpos", 1000, -10, 50, 200, -100, 100);

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
   Double_t m_C12 = 12.00 * 931.49401;
   Double_t m_C13 = 13.00335484 * 931.49401;
   Double_t m_C14 = 14.003242 * 931.49401;
   Double_t m_C15 = 15.0105993 * 931.49401;
   Double_t m_C16 = 16.0147 * 931.49401;
   Double_t m_C17 = 17.0226 * 931.49401;

   Double_t m_a = 4.00260325415 * 931.49401;
   Double_t m_O16 = 15.99491461956 * 931.49401;

   Double_t Ebeam_buff = 195.0; // 192.0;
   Double_t m_b;
   Double_t m_B;

   m_b = m_p;
   m_B = m_C15;

   Double_t Am = 1;

   std::vector<std::pair<TString, TString>> filepairs;
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
   filepairs.push_back(std::make_pair("run_0148.root", "run_0148_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0149.root", "run_0149_FRIB_sorted.root"));
   // filepairs.push_back(std::make_pair("run_0150.root", "run_0150_FRIB_sorted.root")); // Not existing
   // filepairs.push_back(std::make_pair("run_0151.root", "run_0151_FRIB_sorted.root")); // Wrong tiem stamp matching
   filepairs.push_back(std::make_pair("run_0152.root", "run_0152_FRIB_sorted.root"));
   filepairs.push_back(std::make_pair("run_0153.root", "run_0153_FRIB_sorted.root"));
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

   TCutG *cutt = new TCutG("CUTT", 19);
   cutt->SetVarX("ELossvsBrho");
   cutt->SetVarY("");
   cutt->SetTitle("Graph");
   cutt->SetFillStyle(1000);
   cutt->SetPoint(0, 451.1299, 2.290419);
   cutt->SetPoint(1, 693.7338, 2.054056);
   cutt->SetPoint(2, 831.2093, 1.742258);
   cutt->SetPoint(3, 984.8584, 1.551156);
   cutt->SetPoint(4, 1211.289, 1.304735);
   cutt->SetPoint(5, 1445.806, 1.153865);
   cutt->SetPoint(6, 1801.625, 0.9979663);
   cutt->SetPoint(7, 1914.84, 0.9476764);
   cutt->SetPoint(8, 1979.534, 0.8521254);
   cutt->SetPoint(9, 1817.798, 0.8018354);
   cutt->SetPoint(10, 1518.587, 0.8672124);
   cutt->SetPoint(11, 920.1641, 1.093517);
   cutt->SetPoint(12, 734.1678, 1.299706);
   cutt->SetPoint(13, 321.7412, 1.752316);
   cutt->SetPoint(14, 240.8732, 1.968563);
   cutt->SetPoint(15, 313.6544, 2.265274);
   cutt->SetPoint(16, 443.0431, 2.315564);
   cutt->SetPoint(17, 459.2167, 2.295448);
   cutt->SetPoint(18, 451.1299, 2.290419);

   TCutG *cutd = new TCutG("CUTD", 20);
   cutd->SetVarX("ELossvsBrho");
   cutd->SetVarY("");
   cutd->SetTitle("Graph");
   cutd->SetFillStyle(1000);
   cutd->SetPoint(0, 212.3307, 1.928531);
   cutd->SetPoint(1, 655.0554, 1.352218);
   cutd->SetPoint(2, 1085.131, 1.02354);
   cutd->SetPoint(3, 1338.116, 0.8659547);
   cutd->SetPoint(4, 1894.685, 0.7578961);
   cutd->SetPoint(5, 2691.589, 0.6498374);
   cutd->SetPoint(6, 3437.897, 0.5823008);
   cutd->SetPoint(7, 4475.137, 0.4967544);
   cutd->SetPoint(8, 4867.265, 0.4562324);
   cutd->SetPoint(9, 4867.265, 0.4022031);
   cutd->SetPoint(10, 4348.645, 0.3436714);
   cutd->SetPoint(11, 2565.096, 0.4382227);
   cutd->SetPoint(12, 1376.064, 0.5552862);
   cutd->SetPoint(13, 794.1975, 0.7578961);
   cutd->SetPoint(14, 452.667, 1.001028);
   cutd->SetPoint(15, 174.3829, 1.271174);
   cutd->SetPoint(16, 98.4872, 1.775448);
   cutd->SetPoint(17, 187.0321, 1.960048);
   cutd->SetPoint(18, 212.3307, 1.928531);

   TCutG *cutp = new TCutG("CUTP", 25);
   cutp->SetVarX("ELossvsBrho");
   cutp->SetVarY("");
   cutp->SetTitle("Graph");
   cutp->SetFillStyle(1000);
   cutp->SetPoint(0, 267.3558, 1.026347);
   cutp->SetPoint(1, 540.5584, 1.002354);
   cutp->SetPoint(2, 540.5584, 0.9523692);
   cutp->SetPoint(3, 649.8394, 0.8244083);
   cutp->SetPoint(4, 1005.003, 0.6224699);
   cutp->SetPoint(5, 1879.251, 0.4305285);
   cutp->SetPoint(6, 3245.264, 0.3465542);
   cutp->SetPoint(7, 5540.165, 0.2705774);
   cutp->SetPoint(8, 6933.498, 0.2305896);
   cutp->SetPoint(9, 9310.36, 0.1985994);
   cutp->SetPoint(10, 11823.82, 0.1586116);
   cutp->SetPoint(11, 14446.57, 0.1286207);
   cutp->SetPoint(12, 17342.52, 0.1206232);
   cutp->SetPoint(13, 23844.74, 0.09663049);
   cutp->SetPoint(14, 24664.34, 0.08863293);
   cutp->SetPoint(15, 24664.34, 0.06264087);
   cutp->SetPoint(16, 24117.94, 0.05264392);
   cutp->SetPoint(17, 10812.97, 0.06064148);
   cutp->SetPoint(18, 7261.341, 0.1066274);
   cutp->SetPoint(19, 1988.532, 0.222592);
   cutp->SetPoint(20, 212.7153, 0.386542);
   cutp->SetPoint(21, 158.0748, 0.648462);
   cutp->SetPoint(22, 76.11406, 0.8763924);
   cutp->SetPoint(23, 76.11406, 1.030345);
   cutp->SetPoint(24, 267.3558, 1.026347);

   for (auto iFile : filepairs) {
      // for (auto iFile : files) {

      TFile *file = new TFile(iFile.first.Data(), "READ");
      std::cout << " Processing file : " << iFile.first.Data() << "\n";

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader ReaderTracking("cbmsim", file);
      TTreeReaderValue<TClonesArray> trackingArray(ReaderTracking, "AtTrackingEvent");
      TTreeReaderValue<TClonesArray> eventHArray(ReaderTracking, "AtEventH");

      // FRIB data
      TFile *fileFRIB = new TFile(iFile.second.Data(), "READ");
      TTree *treeFRIB = (TTree *)fileFRIB->Get("FRIB_output_tree");
      Int_t nEventsFRIB = treeFRIB->GetEntries();
      std::cout << " Number of FRIB DAQ events : " << nEventsFRIB << std::endl;

      TTreeReader ReaderAux("FRIB_output_tree", fileFRIB);
      TTreeReaderValue<ULong64_t> ts(ReaderAux, "timestamp");
      TTreeReaderValue<std::vector<Float_t>> energyIC(ReaderAux, "energy");
      TTreeReaderValue<std::vector<Float_t>> timeIC(ReaderAux, "time");
      TTreeReaderValue<UInt_t> multIC(ReaderAux, "mult");
      TTreeReaderValue<std::string> fribEvName(ReaderAux, "eventName");

      if (nEvents != nEventsFRIB) {
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
         ReaderAux.Next();

         AtTrackingEvent *trackingEvent = (AtTrackingEvent *)trackingArray->At(0);
         // AtEvent *event = (AtEvent *)eventHArray->At(0);

         // if (trackingEvent && event) {
         if (trackingEvent) {

            auto getTS = 0;
            // auto getTS = event->GetTimestamp(1);

            if (i == 0) {
               fribDTS = 0;
               getDTS = 0;
            } else {

               fribDTS = *ts - fribTSRef;
               getDTS = getTS - getTSRef;
            }

            /* if ((fribDTS > (getDTS + 5) || fribDTS < (getDTS - 5)) && i != 0) {
                std::cout << " -------------------------------- "
                          << "\n";
                std::cerr << i << "  " << fribDTS << "  " << getDTS << "\n";
                std::cerr << i << "  " << *ts << "  " << getTS << "\n";
                // std::exit(0);
             }*/

            getTSRef = getTS;
            fribTSRef = *ts;

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

               auto [ex_energy, theta_cm] =
                  kine_2b(m_C15, m_p, m_b, m_B, Ebeam_buff, theta * TMath::DegToRad(), energy * Am);

               // Excitation energy vs Beam energy
               for (auto iEb = 0; iEb < 300; ++iEb) {
                  auto [_ex_energy, _theta_cm] =
                     kine_2b(m_C15, m_p, m_b, m_B, iEb, theta * TMath::DegToRad(), energy * Am);
                  QvsEb->Fill(_ex_energy, iEb);
               }

               // Rough vertex
               // QvsZpos->Fill(ex_energy_exp, zpos / 10.0);

               // Conditions
               if (!cutp->IsInside(eLossADC, brho))
                  continue;

               // if (!cutt->IsInside(eLossADC, brho))
               // continue;

               /* if(!cuttdedx->IsInside(dEdxADC, brho))
                   continue;*/

               if (theta > 90.0 || theta < 10.0)
                  continue;

               if (energy * Am > 6.0)
                  continue;

               // Histograms
               Ang_Ener->Fill(theta, energy * Am);
               Ang_Ener_PRAC->Fill(thetaPRA, energyPRA * Am);
               ELossvsBrho->Fill(eLossADC, brho);
               dedxvsBrho->Fill(dEdxADC, brho);
               hex->Fill(ex_energy);

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

   TCanvas *c_ExEner = new TCanvas();
   c_ExEner->Divide(2, 1);
   c_ExEner->cd(1);
   hex->Draw();
   c_ExEner->cd(2);

   TCanvas *c_ExEnerBeam = new TCanvas();
   QvsEb->Draw("zcol");
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
