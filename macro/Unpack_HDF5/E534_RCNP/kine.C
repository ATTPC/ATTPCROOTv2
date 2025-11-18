TGraph* ReadKinematics(TString kineFile);
Double_t omega(Double_t x, Double_t y, Double_t z);
std::tuple<double, double> kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject);

void kine()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   // AtMap to check if a hit belong to a big pad or small pad.
   TString scriptfile = "rcnp_map_size.xml";
   TString dir = getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + scriptfile;
   AtTpcMap *map = new AtTpcMap();
   map->ParseXMLMap(mapDir.Data());
   map->GeneratePadPlane();

   // Punch through filter.
   double punchThroughThreshold = 20;
   AtTools::AtPunchThroughChecker punchThroughChecker = AtTools::AtPunchThroughChecker();
   punchThroughChecker.SetDistanceThreshold(punchThroughThreshold);

   // ELoss model for kinetic energy estimations.
   double density = 1.3378e-3; // 470Torr
   std::vector<std::tuple<int, int, int>> materialComponents;
   materialComponents.push_back(std::make_tuple(12, 6, 3));
   materialComponents.push_back(std::make_tuple(2, 1, 8));

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_p = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_470Torr_p");
   eLossModelC3D8_p->SetMaterial(materialComponents);
   eLossModelC3D8_p->SetProjectile(1, 1, 1.007825031898);
   eLossModelC3D8_p->SetPDGCode("1000010010");

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_d = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_470Torr_d");
   eLossModelC3D8_d->SetMaterial(materialComponents);
   eLossModelC3D8_d->SetProjectile(2, 1, 2.0135532);
   eLossModelC3D8_d->SetPDGCode("1000020010");

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_3He = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_470Torr_3He");
   eLossModelC3D8_3He->SetMaterial(materialComponents);
   eLossModelC3D8_3He->SetProjectile(3, 2, 3.01602932197);
   eLossModelC3D8_3He->SetPDGCode("1000030020");

   // Cut files.
   /*TFile *cutKineFile = new TFile("./cutFiles/cutKine.root", "READ");
   TCutG *cutKineBProton = (TCutG *)cutKineFile->Get("cutKineBProton");
   cutKineFile->Close();*/

   TFile *cutPIDFile = new TFile("./cutFiles/PID.root", "READ");
   TCutG *cutPID3He = (TCutG *)cutPIDFile->Get("cutPID3He");
   cutPIDFile->Close();

   TFile *cutSiPIDFile = new TFile("./cutFiles/SiPID.root", "READ");
   TCutG *cutSiN = (TCutG *)cutSiPIDFile->Get("cutSiPIDN");
   TCutG *cutSiC = (TCutG *)cutSiPIDFile->Get("cutSiPIDC");
   cutSiPIDFile->Close();

   TFile *cutGaggPIDFile = new TFile("./cutFiles/GAGGPID.root", "READ");
   TCutG *cutGaggC = (TCutG *)cutGaggPIDFile->Get("cutGAGGC");
   cutGaggPIDFile->Close();

   // Histogram definitions.
   TH2F *histRangeVThetaLAB = new TH2F("histRangeVThetaLAB", "histRangeVThetaLAB", 180, 0, 180, 1030, 0, 1030);
   TH2F *histEstimatedKinEVThetaLABTotal = new TH2F("histEstimatedKinEVThetaLABTotal", "histEstimatedKinEVThetaLABTotal", 360, 0, 180, 500, 0, 20);
   TH2F *histEstimatedKinEVThetaLAB3He = new TH2F("histEstimatedKinEVThetaLAB3He", "histEstimatedKinEVThetaLAB3He", 180, 0, 180, 250, 0, 20);
   TH2F *histdEdxVTotalRange = new TH2F("histdEdxVTotalRange", "histdEdxVTotalRange", 500, 0, 1030, 1600, 0, 4000);
   TH2F *histESmallVTotalRange = new TH2F("histESmallVTotalRange", "histESmallVTotalRange", 500, 0, 1030, 1600, 0, 160000);
   TH2F *histEBigVBigRange = new TH2F("histEBigVBigRange", "histEBigVBigRange", 500, 0, 1030, 1600, 0, 160000);

   TH2F *histSiPIDTraceIntegral = new TH2F("histSiPIDTraceIntegral", "histSiPIDTraceIntegral", 4000, 0, 90000, 4000, 0, 140000);
   TH2F *histSiPIDADCMax = new TH2F("histSiPIDADCMax", "histSiPIDADCMax", 4000, 0, 4000, 4000, 0, 4000);
   TH1F *histSiMultiplicityFront1 = new TH1F("histSiMultiplicityFront1", "histSiMultiplicityFront1", 5, 0, 5);
   TH1F *histSiMultiplicityFront2 = new TH1F("histSiMultiplicityFront2", "histSiMultiplicityFront2", 5, 0, 5);

   TH1F *histGaggMultiplicity1 = new TH1F("histGaggMultiplicity1", "histGaggMultiplicity1", 25, 0, 25);
   TH1F *histGaggMultiplicity2 = new TH1F("histGaggMultiplicity2", "histGaggMultiplicity2", 16, 0, 16);
   TH2F *histGaggPIDADCMax = new TH2F("histGaggPIDADCMax", "histGaggPIDADCMax", 5000, 0, 10000, 4000, 0, 4000);

   // All events?
   std::vector runNums = {2009, 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
                          2020, 2021, 2022, 2023, 2024, 2025, 2026, 2027, 2028, 2029, 2030
                          };

   // Only events with good processed GAGG data in them.
   //std::vector runNums = {2034};//, 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019,
                          //2020, 2021, 2022, 2023, 2024, 2025, 2026, 2027, 2028, 2029, 2030};

   for (int runNum: runNums) {
      // Open the digitalization file and get the TTree.
      TString unpackFileName = TString::Format("/data/ATTPCROOTv2_results/E534/UnpackerOutput/run_%04d.root", runNum);
      TFile *unpackFile = new TFile(unpackFileName, "READ");
      TTree *unpackTree = (TTree *)unpackFile->Get("cbmsim");
      int nUnpackEvents = unpackTree->GetEntries();
      std::cout << " Number of unpacked events in run " << runNum << ": " << nUnpackEvents << std::endl;

      // Creare the TTreeReader to read the AtTrackingEvents and simulation.
      TTreeReader unpackReader("cbmsim", unpackFile);
      TTreeReaderValue<TClonesArray> siArray(unpackReader, "AtSiEvent");
      //TTreeReaderValue<TClonesArray> gaggArray(unpackReader, "AtGaggEvent");
      TTreeReaderValue<TClonesArray> patternArray(unpackReader, "AtPatternEvent");

      // Open the GAGG file of this run.
      TString gaggFileName = TString::Format("/data/sustech/user/zzc/frib/frib-decode/data/hit%04d.root", runNum);
      TFile *gaggFile = new TFile(gaggFileName, "READ");
      TTree *gaggTree = (TTree *)gaggFile->Get("tree");
      int nGaggEvents = gaggTree->GetEntries();
      std::cout << " Number of unpacked GAGG events in run " << runNum << ": " << nGaggEvents << std::endl;

      Int_t idGagg1[25];
      Int_t idGagg2[16];
      Double_t gaggADCMax1[25];
      Double_t gaggADCMax2[16];

      gaggTree->SetBranchAddress("id_g1", idGagg1);
      gaggTree->SetBranchAddress("id_g2", idGagg2);
      gaggTree->SetBranchAddress("ADC_max_g1", gaggADCMax1);
      gaggTree->SetBranchAddress("ADC_max_g2", gaggADCMax2);

      // Loop over events.
      //for (int i = 0; i < nUnpackEvents; i++) {
      for (int i = 0; i < nGaggEvents; i++) {
         unpackReader.Next();
         gaggTree->GetEntry(i);

         // Check the Si data first.
         AtSiEvent *siEvent = (AtSiEvent *)siArray->At(0);

         Int_t multiplicityFront1 = siEvent->GetMultiplicityFront1();
         Int_t multiplicityFront2 = siEvent->GetMultiplicityFront2();

         histSiMultiplicityFront1->Fill(multiplicityFront1);
         histSiMultiplicityFront2->Fill(multiplicityFront2);

         if (multiplicityFront1 != 1 || multiplicityFront2 != 1) continue;

         Double_t maxADCFront1 = siEvent->GetADCMaxFront1(0);
         Double_t maxADCFront2 = siEvent->GetADCMaxFront2(0);

         Double_t EFront1 = siEvent->GetEFront1(0);
         Double_t EFront2 = siEvent->GetEFront2(0);

         histSiPIDTraceIntegral->Fill(EFront2, EFront1);
         histSiPIDADCMax->Fill(maxADCFront2, maxADCFront1);

         //if (!cutSiN->IsInside(maxADCFront2, maxADCFront1)) continue;
         if (!cutSiC->IsInside(maxADCFront2, maxADCFront1)) continue;

         // Check the GAGG data directly from the GAGG decoder.
         Double_t totalMaxADCGagg1{};
         for (int j = 0; j < 25; j++) {
            if (idGagg1[j])
               totalMaxADCGagg1 += gaggADCMax1[j];
         }

         Double_t totalMaxADCGagg2{};
         for (int j = 0; j < 16; j++) {
            if (idGagg1[j])
               totalMaxADCGagg2 += gaggADCMax2[j];
         }

         Double_t totalMaxADCGagg = totalMaxADCGagg1 + totalMaxADCGagg2;
         if (totalMaxADCGagg)
            histGaggPIDADCMax->Fill(totalMaxADCGagg, maxADCFront2);

         if (!cutGaggC->IsInside(totalMaxADCGagg, maxADCFront2)) continue;

         // Check the GAGG data
         /*AtGaggEvent *gaggEvent = (AtGaggEvent *)gaggArray->At(0);

         Int_t multiGagg1 = gaggEvent->GetMultiplicity1();
         Int_t multiGagg2 = gaggEvent->GetMultiplicity2();

         histGaggMultiplicity1->Fill(multiGagg1);
         histGaggMultiplicity2->Fill(multiGagg2);

         std::cout << " Multiplicity GAGG1: " << multiGagg1 <<std::endl;

         //if (multiGagg1 != 1) continue;
         Double_t maxADCGagg1, maxADCGagg2;
         Double_t gaggADC1{-1}, gaggADC2{-1};
         Double_t totalMaxADCGagg{};
         for(int imul = 0; imul < multiGagg1; imul++) {
            maxADCGagg1 = gaggEvent->GetADCMax1(imul);
            gaggADC1 += maxADCGagg1;
            totalMaxADCGagg += maxADCGagg1;
            std::cout << " ADC entry = " << maxADCGagg1 << std::endl;
         }
	      for(int imul = 0; imul < multiGagg2; imul++) {
            maxADCGagg2 = gaggEvent->GetADCMax2(imul);
            gaggADC2 += maxADCGagg2;
            totalMaxADCGagg += maxADCGagg2;
         }
         Double_t totalMaxADCGagg{};
         if (gaggADC1 != -1)
            totalMaxADCGagg += gaggADC1;
         if (gaggADC2 != -1)
            totalMaxADCGagg += gaggADC2;

         histGaggPIDADCMax->Fill(totalMaxADCGagg, maxADCFront2);*/


         // First, we obtain some rough kinematics just by using the AtPatternEvent.
         AtPatternEvent *patternEvent = (AtPatternEvent *)patternArray->At(0);
         if (!patternEvent) continue;

         // We want to focus on events with 2 or less tracks for now.
         auto &tracks = patternEvent->GetTrackCand();
         //if (tracks.size() > 4) continue;
         if (tracks.size() > 2) continue;
         //if (tracks.size() > 1) continue;

         // Iterate over AtTracks and extract their kinematics.
         for (auto &track: tracks) {

            bool isPunchThrough = punchThroughChecker.IsPunchThrough(&track);
            if (isPunchThrough) continue;

            auto *pattern = track.GetPattern();

            auto firstPoint = track.GetFirstPoint();
            auto lastPoint = track.GetLastPoint();
	    //cut penetrate particles
	    //if (lastPoint.Z()>900) continue;
	    //std::cout<<lastPoint.Z()<<std::endl;
            double roughRangeEstimation = pattern->DistanceAlongPattern(lastPoint, firstPoint);

            double trackThetaLAB = track.GetGeoTheta() * 180 / TMath::Pi();
            double trackPhi = track.GetGeoPhi() * 180 / TMath::Pi();

            double smallPadCharge{};
            double bigPadCharge{};
            auto braggCurvePairs = track.GetBraggCurveValues();
            auto &hits = track.GetHitArray();
            double rangeInSmallPads{};
	         for (auto &hit: hits) {
               int padNum = hit->GetPadNum();
               int sizeID = map->GetPadSize(padNum);
               if (sizeID == 1) {
                  bigPadCharge += hit->GetCharge();
                  continue;
               }
               smallPadCharge += hit->GetCharge();
               double currentRangeInSmallPads = pattern->DistanceAlongPattern(hit->GetPosition(), firstPoint);
               if (currentRangeInSmallPads > rangeInSmallPads)
                  rangeInSmallPads = currentRangeInSmallPads;
            }
            double dEdx = smallPadCharge / rangeInSmallPads;

            double rangeInBigPads = roughRangeEstimation - rangeInSmallPads;
            bool reachedBigPads = true;
            if (rangeInBigPads / roughRangeEstimation < 0.05)
               reachedBigPads = false;

            double estimatedKinE{0.1};
            if (cutPID3He->IsInside(roughRangeEstimation, dEdx)) {
               while (eLossModelC3D8_3He->GetRange(estimatedKinE) < roughRangeEstimation)
                  estimatedKinE += 0.01;
            } else {
               while (eLossModelC3D8_d->GetRange(estimatedKinE) < roughRangeEstimation)
                  estimatedKinE += 0.01;
            }

            histRangeVThetaLAB->Fill(trackThetaLAB, roughRangeEstimation);
            histdEdxVTotalRange->Fill(roughRangeEstimation, dEdx);
            histEstimatedKinEVThetaLABTotal->Fill(trackThetaLAB, estimatedKinE);

            if (cutPID3He->IsInside(roughRangeEstimation, dEdx))
               histEstimatedKinEVThetaLAB3He->Fill(trackThetaLAB, estimatedKinE);

            if (!reachedBigPads)
               histESmallVTotalRange->Fill(roughRangeEstimation, smallPadCharge);
            else
               histEBigVBigRange->Fill(rangeInBigPads, bigPadCharge);

         }
      }
      // Close files.
      unpackFile->Close();
      gaggFile->Close();
   }

   // Kinematic lines.
   TGraph *kine_dd = ReadKinematics("./kineFiles/17N_dd_gs.txt");
   TGraph *kine_d3He = ReadKinematics("./kineFiles/17N_d3He_gs.txt");
   TGraph *kine_12C12C = ReadKinematics("./kineFiles/17N_12C12C_gs.txt");

   // Draw histograms in TCanvas.
   TCanvas *c = new TCanvas();
   histRangeVThetaLAB->Draw("zcol");
   histRangeVThetaLAB->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histRangeVThetaLAB->GetYaxis()->SetTitle("roughRange [mm]");

   TCanvas *c2 = new TCanvas();
   histEstimatedKinEVThetaLABTotal->Draw("zcol");
   kine_dd->Draw("same");
   kine_d3He->Draw("same");
   kine_12C12C->Draw("same");
   histEstimatedKinEVThetaLABTotal->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLABTotal->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c3 = new TCanvas();
   histEstimatedKinEVThetaLAB3He->Draw("zcol");
   kine_dd->Draw("same");
   kine_d3He->Draw("same");
   kine_12C12C->Draw("same");
   histEstimatedKinEVThetaLAB3He->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLAB3He->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c4 = new TCanvas();
   histdEdxVTotalRange->Draw("zcol");
   cutPID3He->Draw("same");
   histdEdxVTotalRange->GetXaxis()->SetTitle("roughRange [mm]");
   histdEdxVTotalRange->GetYaxis()->SetTitle("#frac{dE}{dx} [ADC/mm]");

   TCanvas *c5 = new TCanvas();
   histESmallVTotalRange->Draw("zcol");
   histESmallVTotalRange->GetXaxis()->SetTitle("smallRange [mm]");
   histESmallVTotalRange->GetYaxis()->SetTitle("E^{small}_{Loss} [ADC]");

   TCanvas *c6 = new TCanvas();
   histEBigVBigRange->Draw("zcol");
   histEBigVBigRange->GetXaxis()->SetTitle("bigRange [mm]");
   histEBigVBigRange->GetYaxis()->SetTitle("E^{big}_{Loss} [ADC]");

   TCanvas *c7 = new TCanvas();
   histSiPIDTraceIntegral->Draw("zcol");
   histSiPIDTraceIntegral->GetXaxis()->SetTitle("E2 [ADC]");
   histSiPIDTraceIntegral->GetYaxis()->SetTitle("E1 [ADC]");

   TCanvas *c8 = new TCanvas();
   histSiPIDADCMax->Draw("zcol");
   cutSiN->Draw("same");
   cutSiC->Draw("same");
   histSiPIDADCMax->GetXaxis()->SetTitle("ADC^{max}_{2} [ADC]");
   histSiPIDADCMax->GetYaxis()->SetTitle("ADC^{max}_{1} [ADC]");

   TCanvas *c9 = new TCanvas();
   histSiMultiplicityFront1->Draw();

   TCanvas *c10 = new TCanvas();
   histSiMultiplicityFront2->Draw();

   TCanvas *c11 = new TCanvas();
   histGaggPIDADCMax->Draw("colz");
   cutGaggC->Draw("same");
   histGaggPIDADCMax->GetXaxis()->SetTitle("#Sigma ADC^{max}_{Gagg} [ADC]");
   histGaggPIDADCMax->GetYaxis()->SetTitle("ADC^{max}_{2} [ADC]");

   TCanvas *c12 = new TCanvas();
   histGaggMultiplicity1->Draw();

   TCanvas *c13 = new TCanvas();
   histGaggMultiplicity2->Draw();
}

TGraph* ReadKinematics(TString kineFile)
{
   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   std::ifstream *kineStr = new std::ifstream(kineFile.Data());
   Int_t numKin = 0;

   if (!kineStr->fail()){
      while (!kineStr->eof()){
         *kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >>
                     ThetaLabSca[numKin] >> EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *kine = new TGraph(numKin, ThetaLabRec, EnerLabRec);
   return kine;
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

   /*theta_cm = acos((pow(s, 2) + s * (2 * u - pow(m1, 2) - pow(m2, 2) - pow(m3, 2) - pow(m4_ex, 2)) +
                                  (pow(m1, 2) - pow(m2, 2)) * (pow(m4_ex, 2) - pow(m3, 2))) /
                                 (omega(s, pow(m1, 2), pow(m2, 2)) * omega(s, pow(m4_ex, 2), pow(m3, 2))));*/

   theta_cm = theta_cm * TMath::RadToDeg();
   return std::make_tuple(Ex, theta_cm);
}
