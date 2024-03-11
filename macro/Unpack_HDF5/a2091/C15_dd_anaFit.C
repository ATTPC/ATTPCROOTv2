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

void C15_dd_anaFit()
{
   FairRunAna *run = new FairRunAna();

   TH2F *Ang_Ener = new TH2F("Ang_Ener", "Ang_Ener", 720, 0, 179, 1000, 0, 200.0);
   TH2F *Ang_Ener_PRAC = new TH2F("Ang_Ener_PRAC", "Ang_Ener_PRAC", 1000, 0, 100, 1000, 0, 200.0);
   TH2F *ELossvsBrho = new TH2F("ELossvsBrho", "ELossvsBrho", 4000, 0, 25000, 1000, 0, 4);
   TH2F *dedxvsBrho = new TH2F("dedxvsBrho", "dedxvsBrho", 4000, 0, 4000000, 1000, 0, 4);
   TH2F *hVxVy = new TH2F("hVxVy", "hVxVy", 1000, 0, 10, 1000, 0, 10);

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

   Double_t Am = 2;

   std::vector<TString> files{"run_0013.root"};
   files.push_back("run_0014.root");
   files.push_back("run_0015.root");
   files.push_back("run_0016.root");
   files.push_back("run_0017.root");
   files.push_back("run_0018.root");
   files.push_back("run_0019.root");
   files.push_back("run_0022.root");
   files.push_back("run_0023.root");
   files.push_back("run_0024.root");
   files.push_back("run_0026.root");
   files.push_back("run_0027.root");
   files.push_back("run_0028.root");
   files.push_back("run_0029.root");
   files.push_back("run_0030.root");
   files.push_back("run_0031.root");
   files.push_back("run_0032.root");
   files.push_back("run_0033.root");
   files.push_back("run_0034.root");
   files.push_back("run_0035.root");
   files.push_back("run_0036.root");
   files.push_back("run_0037.root");
   files.push_back("run_0038.root");
   files.push_back("run_0039.root");
   files.push_back("run_0040.root");
   files.push_back("run_0042.root");
   files.push_back("run_0043.root");
   files.push_back("run_0044.root");
   files.push_back("run_0049.root");
   files.push_back("run_0050.root");
   files.push_back("run_0051.root");
   files.push_back("run_0052.root");
   files.push_back("run_0053.root");
   files.push_back("run_0054.root");
   files.push_back("run_0055.root");
   files.push_back("run_0056.root");
   files.push_back("run_0058.root");
   files.push_back("run_0059.root");
   files.push_back("run_0060.root");
   files.push_back("run_0061.root");
   files.push_back("run_0062.root");
   files.push_back("run_0063.root");
   files.push_back("run_0064.root");
   files.push_back("run_0065.root");
   files.push_back("run_0066.root");
   files.push_back("run_0067.root");
   files.push_back("run_0068.root");
   files.push_back("run_0069.root");
   files.push_back("run_0070.root");
   files.push_back("run_0071.root");
   files.push_back("run_0072.root");
   files.push_back("run_0073.root");
   files.push_back("run_0074.root");
   files.push_back("run_0075.root");

   TCutG *cutt = new TCutG("CUTT", 10);
   cutt->SetVarX("ELossvsBrho");
   cutt->SetVarY("");
   cutt->SetTitle("Graph");
   cutt->SetFillStyle(1000);
   cutt->SetPoint(0, 116.0509, 2.755236);
   cutt->SetPoint(1, 161.867, 1.927617);
   cutt->SetPoint(2, 439.1129, 1.341213);
   cutt->SetPoint(3, 527.2208, 1.299624);
   cutt->SetPoint(4, 655.2708, 1.465979);
   cutt->SetPoint(5, 658.7951, 1.865233);
   cutt->SetPoint(6, 402.695, 2.405889);
   cutt->SetPoint(7, 219.4308, 2.821779);
   cutt->SetPoint(8, 113.7014, 2.755236);
   cutt->SetPoint(9, 116.0509, 2.755236);

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
               /*if (!cutd->IsInside(eLossADC, brho))
                continue;*/

               /* if(!cuttdedx->IsInside(dEdxADC, brho))
                   continue;*/

               if (theta > 90.0 || theta < 10.0)
                  continue;

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
