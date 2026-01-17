TGraph* ReadKinematics(TString kineFile);
Double_t omega(Double_t x, Double_t y, Double_t z);
std::tuple<double, double> kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject);

void kine()
{

   // Masses.
   double u_to_MeV = 931.49401;
   double m_17C = 17.022578650 * u_to_MeV;
   double m_18C = 18.026751930 * u_to_MeV;
   double m_p = 1.007825031898 * u_to_MeV;
   double m_d = 2.014101777844 * u_to_MeV;

   // Beam energy.
   double E_beam = 27.091 * 17.022578650;

   FairRunAna *run = new FairRunAna(); // Forcing a dummy run
   //   TString outfname="./canvas_kine.root";
   //   TFile *outfile=new TFile(outfname,"recreate");

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
   // ... ATTPC
   double density = 1.3213e-3; // 464.2Torr
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

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_17C = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_470Torr_17C");
   eLossModelC3D8_17C->SetMaterial(materialComponents);
   eLossModelC3D8_17C->SetProjectile(17, 6, 17.022578650);
   eLossModelC3D8_17C->SetPDGCode("1000170060");

   // Cut files.
   /*TFile *cutKineFile = new TFile("./cutFiles/cutKine.root", "READ");
   TCutG *cutKineBProton = (TCutG *)cutKineFile->Get("cutKineBProton");
   cutKineFile->Close();*/

   TFile *cutATTPCPIDFile = new TFile("./cutFiles/ATTPC_PID.root", "READ"); 
   TCutG *cutPIDproton = (TCutG *)cutATTPCPIDFile->Get("cutATTPCPIDproton");
   TCutG *cutPIDproton_extension = (TCutG *)cutATTPCPIDFile->Get("cutATTPCPIDproton_extension");
   TCutG *cutPIDdeuteron = (TCutG *)cutATTPCPIDFile->Get("cutATTPCPIDdeuteron");
   cutATTPCPIDFile->Close();
   
   TFile *cutSiPIDFile = new TFile("./cutFiles/SiPID.root", "READ");
   TCutG *cutSiB = (TCutG *)cutSiPIDFile->Get("cutSiPIDB");
   TCutG *cutSiC = (TCutG *)cutSiPIDFile->Get("cutSiPIDC");
   TCutG *cutSiN = (TCutG *)cutSiPIDFile->Get("cutSiPIDN");
   cutSiPIDFile->Close();
/*
   TFile *cutGaggPIDFile = new TFile("./cutFiles/GAGGPID.root", "READ");
   TCutG *cutGaggB = (TCutG *)cutGaggPIDFile->Get("cutGAGGPIDB");
   TCutG *cutGaggBe = (TCutG *)cutGaggPIDFile->Get("cutGAGGPIDBe");
   TCutG *cutGaggLi = (TCutG *)cutGaggPIDFile->Get("cutGAGGPIDLi");
   TCutG *cutGaggHe = (TCutG *)cutGaggPIDFile->Get("cutGAGGPIDHe");
   cutGaggPIDFile->Close();*/

   // Kinematic curve
   /*TGraph* kinecurve_3HeGS = new TGraph("ang_lab_cm_3HeGS.txt","%lg %*s %lg");
   kinecurve_3HeGS->SetLineWidth(2);
   kinecurve_3HeGS->SetLineColor(kRed);
   TGraph* ang_lab_cm_3HeGS = new TGraph("ang_lab_cm_3HeGS.txt","%lg %*s %lg");// 5 deg pitch in theta_cm
   ang_lab_cm_3HeGS->SetMarkerStyle(8);
   ang_lab_cm_3HeGS->SetMarkerSize(1);
   TLegend *legend = new TLegend(0.6,0.2,0.85,0.5);
   legend->AddEntry(kinecurve_3HeGS,"3He G.S.","l");
   legend->AddEntry(ang_lab_cm_3HeGS,"5 deg pitch in #theta_{cm}","p");
   legend->SetFillColor(0);*/
   
   // Histogram definitions.
   // ... ATTPC PID
   TH2F *histdEdxVTotalRange = new TH2F("histdEdxVTotalRange", "histdEdxVTotalRange;roughRange [mm];roughRange [mm]", 500, 0, 1030, 1600, 0, 4000);
   TH2F *histdEdxVTotalRangeBackwards = new TH2F("histdEdxVTotalRangeBackwards", "histdEdxVTotalRangeBackwards;roughRange [mm];roughRange [mm]", 500, 0, 1030, 1600, 0, 4000);
   // ... kinematics 
   TH2F *histEstimatedKinEVThetaLABTotal = new TH2F("histEstimatedKinEVThetaLABTotal", "histEstimatedKinEVThetaLABTotal;#theta_{LAB} [deg];roughKinE [MeV]", 180, 0, 180, 300, 0, 30);
   TH2F *histEstimatedKinEVThetaLAB_ProtonATTPC = new TH2F("histEstimatedKinEVThetaLAB_ProtonATTPC", "histEstimatedKinEVThetaLAB_ProtonATTPC;#theta_{LAB} [deg];roughKinE [MeV]", 180, 0, 180, 300, 0, 30);
   TH2F *histEstimatedKinEVThetaLAB_ProtonATTPC_extended = new TH2F("histEstimatedKinEVThetaLAB_ProtonATTPC_extended", "histEstimatedKinEVThetaLAB_ProtonATTPC_extended;#theta_{LAB} [deg];roughKinE [MeV]", 180, 0, 180, 300, 0, 30);
   TH2F *histEstimatedKinEVThetaLAB_DeuteronATTPC = new TH2F("histEstimatedKinEVThetaLAB_DeuteronATTPC", "histEstimatedKinEVThetaLAB_DeuteronATTPC;#theta_{LAB} [deg];roughKinE [MeV]", 180, 0, 180, 300, 0, 30);
   TH2F *histEstimatedKinEVThetaLAB_CarbonSi = new TH2F("histEstimatedKinEVThetaLAB_CarbonSi", "histEstimatedKinEVThetaLAB_CarbonSi;#theta_{LAB} [deg];roughKinE [MeV]", 180, 0, 180, 300, 0, 30);
   TH2F *histEstimatedKinEVThetaLAB_NitrogenSi = new TH2F("histEstimatedKinEVThetaLAB_NitrogenSi", "histEstimatedKinEVThetaLAB_NitrogenSi;#theta_{LAB} [deg];roughKinE [MeV]", 180, 0, 180, 300, 0, 30);
   
   TH2F *histEstimatedKinEVThetaLAB_ProtonATTPC_CarbonSi = new TH2F("histEstimatedKinEVThetaLAB_ProtonATTPC_CarbonSi", "histEstimatedKinEVThetaLAB_ProtonATTPC_CarbonSi;#theta_{LAB} [deg];roughKinE [MeV]", 180, 0, 180, 300, 0, 30);
   

   // ... Excitation energy 
   TH1F *histExdp = new TH1F("histExdp", "histExdp;Ex [MeV]", 80, -5, 15);
   TH1F *histExdp_extended = new TH1F("histExdp_extended", "histExdp_extended;Ex [MeV]", 80, -5, 15);
   TH1F *histExdp_CarbonSi = new TH1F("histExdp_CarbonSi", "histExdp_CarbonSi;Ex [MeV]", 80, -5, 15);
   TH1F *histExdd = new TH1F("histExdd", "histExdd;Ex [MeV]",80 , -5, 15);

   // ... center-of-mass angular distributions
   TH1F *histAngDist_elastic = new TH1F("histAngDist_elastic", "histAngDist_elastic;#theta_{c.m.} [deg]", 180, 0, 180);
   TH1F *histAngDist_dp = new TH1F("histAngDist_dp", "histAngDist_dp;#theta_{c.m.} [deg];Counts / deg", 180, 0, 180);
   TH1F *histAngDist_dp_CarbonSi = new TH1F("histAngDist_dp_CarbonSi", "histAngDist_dp_CarbonSi;#theta_{c.m.} [deg];Counts / deg", 180, 0, 180);

   // ... Si PID
   TH2F *histSiPIDADCMax = new TH2F("histSiPIDADCMax", "histSiPIDADCMax", 4000, 0, 4000, 4000, 0, 4000);
   TH2F *histSiPIDADCMax_ProtonATTPC = new TH2F("histSiPIDADCMax_ProtonATTPC", "histSiPIDADCMax_ProtonATTPC", 4000, 0, 4000, 4000, 0, 4000);
   TH2F *histSiPIDADCMax_DeuteronATTPC = new TH2F("histSiPIDADCMax_DeuteronATTPC", "histSiPIDADCMax_DeuteronATTPC", 4000, 0, 4000, 4000, 0, 4000);
   TH2F *histSiPIDADCMax_TritonATTPC = new TH2F("histSiPIDADCMax_TritonATTPC", "histSiPIDADCMax_TritonATTPC", 4000, 0, 4000, 4000, 0, 4000);
   // ... GAGG
   TH1F *histGaggMultiplicity1 = new TH1F("histGaggMultiplicity1", "histGaggMultiplicity1", 25, 0, 25);
   TH1F *histGaggMultiplicity2 = new TH1F("histGaggMultiplicity2", "histGaggMultiplicity2", 16, 0, 16);
   TH2F *histGaggPIDADCMax = new TH2F("histGaggPIDADCMax", "histGaggPIDADCMax", 5000, 0, 10000, 4000, 0, 4000);
   // ... others ...
   TH2F *histThetaLABThetaLAB = new TH2F("histThetaLABThetaLAB", "histThetaLABThetaLAB", 360, 0, 180, 360, 0, 180);
   TH2F *histESmallVTotalRange = new TH2F("histESmallVTotalRange", "histESmallVTotalRange", 500, 0, 1030, 1600, 0, 160000);
   TH2F *histEBigVBigRange = new TH2F("histEBigVBigRange", "histEBigVBigRange", 500, 0, 1030, 1600, 0, 160000);
   TH2F *histRangeVThetaLAB = new TH2F("histRangeVThetaLAB", "histRangeVThetaLAB", 180, 0, 180, 1030, 0, 1030);
   TH2F *histEstimatedKinEVThetaLAB2H  = new TH2F("histEstimatedKinEVThetaLAB2H", "histEstimatedKinEVThetaLAB2H", 180, 0, 180, 250, 0, 20);
   TH2F *histEstimatedKinEVThetaLAB1H  = new TH2F("histEstimatedKinEVThetaLAB1H", "histEstimatedKinEVThetaLAB1H", 180, 0, 180, 250, 0, 20);
   TH2F *histSiPIDTraceIntegral = new TH2F("histSiPIDTraceIntegral", "histSiPIDTraceIntegral", 4000, 0, 90000, 4000, 0, 140000);
   TH1F *histSiMultiplicityFront1 = new TH1F("histSiMultiplicityFront1", "histSiMultiplicityFront1", 5, 0, 5);
   TH1F *histSiMultiplicityFront2 = new TH1F("histSiMultiplicityFront2", "histSiMultiplicityFront2", 5, 0, 5);




   // All events.
   std::vector runNums = {5109, 5110, 5111, 5112, 5113, 5114, 5115, 5116};

   // unpacked runs
      // 5017, 5018, 5019, 5020, 5021, 5022, 5023, 5024, 5025, 5026, 5027, 5028, 5029, 5030, 5032,
      // 5033, 5034, 5035, 5036, 5037, 5038, 5039, 5040, 5041, 5042, 5043, 5045, 5046, 5047, 
      // 5048, 5049, 5050, 5051, 5052, 5053, 5054, 5055, 5056, 5060, 5061, 5062, 5063, 5064
      // 5065, 5066, 5067, 5068, 5069, 5070, 5071, 5072, 5073, 5074, 5075, 5076, 5077, 5078
      // 5079, 5082, 5083, 5085, 5086, 5087, 5088, 5089, 5090, 5092, 5093, 5094, 5095, 5096
      // 5097, 5099, 5100, 5101, 5102, 5103, 5104, 5105
      // 5109, 5110, 5111, 5112, 5113, 5114, 5115, 5116

   for (int runNum: runNums) {
      // Open the digitalization file and get the TTree.
      TString unpackFileName = TString::Format("/data/ATTPCROOTv2_results/E510/UnpackerOutput/run_%04d_reUnpack_Theshold30.root", runNum);
      TFile *unpackFile = new TFile(unpackFileName, "READ");
      TTree *unpackTree = (TTree *)unpackFile->Get("cbmsim");
      int nUnpackEvents = unpackTree->GetEntries();
      std::cout << " Number of unpacked events in run " << runNum << ": " << nUnpackEvents << std::endl;
      int nEventsWith2Tracks = 0;
      int nEventsWith3He = 0;
      // Creare the TTreeReader to read the AtTrackingEvents and simulation.
      TTreeReader unpackReader("cbmsim", unpackFile);
      TTreeReaderValue<TClonesArray> siArray(unpackReader, "AtSiEvent");
      TTreeReaderValue<TClonesArray> gaggArray(unpackReader, "AtGaggEvent");
      TTreeReaderValue<TClonesArray> patternArray(unpackReader, "AtPatternEvent");

      // Loop over events.
      for (int i = 0; i < nUnpackEvents; i++) {
         unpackReader.Next();

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
         //if (!cutSiC->IsInside(maxADCFront2, maxADCFront1)) continue;

         // Check the GAGG data
         AtGaggEvent *gaggEvent = (AtGaggEvent *)gaggArray->At(0);

         Int_t multiGagg1 = gaggEvent->GetMultiplicity1();
         Int_t multiGagg2 = gaggEvent->GetMultiplicity2();

         histGaggMultiplicity1->Fill(multiGagg1);
         histGaggMultiplicity2->Fill(multiGagg2);

         //if (multiGagg1 != 1) continue;
         Double_t maxADCGagg1, maxADCGagg2;
         Double_t gaggADC1{-1}, gaggADC2{-1};
         Double_t totalMaxADCGagg{};
         for(int imul = 0; imul < multiGagg1; imul++) {
            maxADCGagg1 = gaggEvent->GetADCMax1(imul);
            gaggADC1 += maxADCGagg1;
            totalMaxADCGagg += maxADCGagg1;
         }
	      for(int imul = 0; imul < multiGagg2; imul++) {
            maxADCGagg2 = gaggEvent->GetADCMax2(imul);
            gaggADC2 += maxADCGagg2;
            totalMaxADCGagg += maxADCGagg2;
         }
	      histGaggPIDADCMax->Fill(totalMaxADCGagg, maxADCFront2);

         // First, we obtain some rough kinematics just by using the AtPatternEvent.
         AtPatternEvent *patternEvent = (AtPatternEvent *)patternArray->At(0);
         if (!patternEvent) continue;

         // We want to focus on events with 2 or less tracks for now.
         auto &tracks = patternEvent->GetTrackCand();
         int maxTrackNum{4};
         if (tracks.size() > maxTrackNum) continue;

	      if (tracks.size() == 2) nEventsWith2Tracks++;
	      int trackIndex = 0;
	      double thetaLABArray[2] = {0.0,0.0};
	      double estimatedKineEArray[2] = {0.0,0.0};
	      bool isHe3Array[2] = {false,false};

         // Iterate over AtTracks and extract their kinematics.
         for (auto &track: tracks) {
            bool isPunchThrough = punchThroughChecker.IsPunchThrough(&track);
            // if (isPunchThrough) continue;

            auto *pattern = track.GetPattern();

            auto firstPoint = track.GetFirstPoint();
            auto lastPoint = track.GetLastPoint();
            double roughRangeEstimation = pattern->DistanceAlongPattern(lastPoint, firstPoint);

            auto pseudoVertex = pattern->ClosestPointOnPattern(firstPoint);

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

            histRangeVThetaLAB->Fill(trackThetaLAB, roughRangeEstimation);
            histdEdxVTotalRange->Fill(roughRangeEstimation, dEdx);
            if (!reachedBigPads)
               histESmallVTotalRange->Fill(roughRangeEstimation, smallPadCharge);
            else
               histEBigVBigRange->Fill(rangeInBigPads, bigPadCharge);

            if (trackThetaLAB > 100)
               histdEdxVTotalRangeBackwards->Fill(roughRangeEstimation, dEdx);

            double estimatedKinE{0.1};
            if (cutPIDdeuteron->IsInside(roughRangeEstimation, dEdx)) {
               while (eLossModelC3D8_d->GetRange(estimatedKinE) < roughRangeEstimation)
                  estimatedKinE += 0.01;
	         } else if (cutPIDproton->IsInside(roughRangeEstimation, dEdx) or cutPIDproton_extension->IsInside(roughRangeEstimation, dEdx)) {
               while (eLossModelC3D8_p->GetRange(estimatedKinE) < roughRangeEstimation)
                  estimatedKinE += 0.01;
            } else {
	            estimatedKinE = -1;
            }

            // While we don't have cut files defined, assume deuteron.
            //while (eLossModelC3D8_d->GetRange(estimatedKinE) < roughRangeEstimation)
               //estimatedKinE += 0.01;

            histEstimatedKinEVThetaLABTotal->Fill(trackThetaLAB, estimatedKinE);

            // Kinematics plots
            // .... protons in ATTPC
            if (cutPIDproton->IsInside(roughRangeEstimation, dEdx)) {
               histSiPIDADCMax_ProtonATTPC->Fill(maxADCFront2, maxADCFront1);
               histEstimatedKinEVThetaLAB_ProtonATTPC->Fill(trackThetaLAB, estimatedKinE);

               auto [Ex, thetaCM] = kine_2b(m_17C, m_d, m_p, m_18C, eLossModelC3D8_17C->GetEnergy(E_beam, 1000 - pseudoVertex.z() * 1000), trackThetaLAB * TMath::DegToRad(), estimatedKinE);
               histExdp->Fill(Ex);            
            }
            if (cutPIDproton->IsInside(roughRangeEstimation, dEdx) or cutPIDproton_extension->IsInside(roughRangeEstimation, dEdx)) {
               histEstimatedKinEVThetaLAB_ProtonATTPC_extended->Fill(trackThetaLAB, estimatedKinE);

               auto [Ex, thetaCM] = kine_2b(m_17C, m_d, m_p, m_18C, eLossModelC3D8_17C->GetEnergy(E_beam, 1000 - pseudoVertex.z() * 1000), trackThetaLAB * TMath::DegToRad(), estimatedKinE);
               histExdp_extended->Fill(Ex);

               histAngDist_dp->Fill(thetaCM, 1 / TMath::Sin(thetaCM * TMath::DegToRad()));

               // Also add condition of Carbon in Si
               if (cutSiC->IsInside(maxADCFront2, maxADCFront1)) {
                  histEstimatedKinEVThetaLAB_ProtonATTPC_CarbonSi->Fill(trackThetaLAB, estimatedKinE);
                  histExdp_CarbonSi->Fill(Ex);
                  histAngDist_dp_CarbonSi->Fill(thetaCM, 1 / TMath::Sin(thetaCM * TMath::DegToRad()));
               }
            }
            // .... deuterons in ATTPC
            if (cutPIDdeuteron->IsInside(roughRangeEstimation, dEdx)) {
               histSiPIDADCMax_DeuteronATTPC->Fill(maxADCFront2, maxADCFront1);
               histEstimatedKinEVThetaLAB_DeuteronATTPC->Fill(trackThetaLAB, estimatedKinE);

               auto [Ex, thetaCM] = kine_2b(m_17C, m_d, m_d, m_17C, eLossModelC3D8_17C->GetEnergy(E_beam, 1000 - pseudoVertex.z() * 1000), trackThetaLAB * TMath::DegToRad(), estimatedKinE);
               histExdd->Fill(Ex);

               if ((Ex >= -4.75) && (Ex <= 4.73)) {
                  histAngDist_elastic->Fill(thetaCM, 1 / TMath::Sin(thetaCM * TMath::DegToRad()));
               }
            }
            // .... carbon in Si
            if (cutSiC->IsInside(maxADCFront2, maxADCFront1)) {
               histEstimatedKinEVThetaLAB_CarbonSi->Fill(trackThetaLAB, estimatedKinE);
            }
            // .... nitrogen in Si
            if (cutSiN->IsInside(maxADCFront2, maxADCFront1)) {
               histEstimatedKinEVThetaLAB_NitrogenSi->Fill(trackThetaLAB, estimatedKinE);
            }
         
         }
      }
      //      std::cout << "Number of 2 tracks events in run" << runNum << ":" << nEventsWith2Tracks << std::endl;
      //      std::cout << "Number of events with 3He in run" << runNum << ":" << nEventsWith3He << std::endl;
      // Close files.
      unpackFile->Close();
      //gaggFile->Close();
   }

   // Kinematic lines.
   TGraph *kine_dd_gs = ReadKinematics("./kineFiles/kine17C_dd_gs.txt");
   TGraph *kine_dp_gs = ReadKinematics("./kineFiles/kine17C_dp_gs.txt");

   TGraph *kine_dd_gs_25MeVu = ReadKinematics("./kineFiles/kine17C_dd_gs_25MeVu.txt");
   TGraph *kine_dp_gs_25MeVu = ReadKinematics("./kineFiles/kine17C_dp_gs_25MeVu.txt");

   // Draw histograms in TCanvas.
   TCanvas *c = new TCanvas();
   histRangeVThetaLAB->Draw("zcol");
   histRangeVThetaLAB->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histRangeVThetaLAB->GetYaxis()->SetTitle("roughRange [mm]");
   /*
   TCanvas *c1_3He = new TCanvas();
   histTrackThetaLABRange3He->Draw("zcol");
   histTrackThetaLABRange3He->GetXaxis()->SetTitle("roughRange [mm]");
   histTrackThetaLABRange3He->GetYaxis()->SetTitle("#theta_{LAB} [deg]");
   */
   
   TCanvas *c2 = new TCanvas();
   histEstimatedKinEVThetaLABTotal->Draw("zcol");
   //kine_d3He->Draw("same");
   //kine_d3HeEx2_2->Draw("same");
   //kine_d3HeEx2_7->Draw("same");
   histEstimatedKinEVThetaLABTotal->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLABTotal->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c2_ProtonATTPC = new TCanvas();
   histEstimatedKinEVThetaLAB_ProtonATTPC->Draw("zcol");
   kine_dp_gs->Draw("same");
   kine_dd_gs->Draw("same");
   histEstimatedKinEVThetaLAB_ProtonATTPC->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLAB_ProtonATTPC->GetYaxis()->SetTitle("roughKinE [MeV]");
   
   TCanvas *c2_DeuteronATTPC = new TCanvas();
   histEstimatedKinEVThetaLAB_DeuteronATTPC->Draw("zcol");
   kine_dd_gs->Draw("same");
   histEstimatedKinEVThetaLAB_DeuteronATTPC->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLAB_DeuteronATTPC->GetYaxis()->SetTitle("roughKinE [MeV]");

   

   TCanvas *c3_kineTotal = new TCanvas();
   histEstimatedKinEVThetaLABTotal->Draw("zcol");
   kine_dp_gs->Draw("same");
   kine_dd_gs->Draw("same");
   histEstimatedKinEVThetaLABTotal->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLABTotal->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c3_Exdp = new TCanvas();
   histExdp->Draw();
   histExdp->GetXaxis()->SetTitle("Ex [MeV]");

   TCanvas *c3_Exdd = new TCanvas();
   histExdd->Draw();
   histExdd->GetXaxis()->SetTitle("Ex [MeV]");
   

   TCanvas *c3_tt = new TCanvas();
   histThetaLABThetaLAB->Draw("zcol");
   //kine_d3He_tt->Draw("same");
   histThetaLABThetaLAB->GetXaxis()->SetTitle("track1_#theta_{LAB} [deg]");
   histThetaLABThetaLAB->GetYaxis()->SetTitle("track2_#theta_{LAB} [deg]");

   TCanvas *c4 = new TCanvas();
   histdEdxVTotalRange->Draw("zcol");
   cutPIDproton->Draw("same");
   cutPIDdeuteron->Draw("same");
   histdEdxVTotalRange->GetXaxis()->SetTitle("roughRange [mm]");
   histdEdxVTotalRange->GetYaxis()->SetTitle("roughRange [mm]");

   TCanvas *c4_Backwards = new TCanvas();
   histdEdxVTotalRangeBackwards->Draw("zcol");
   cutPIDproton->Draw("same");
   cutPIDdeuteron->Draw("same");
   histdEdxVTotalRangeBackwards->GetXaxis()->SetTitle("roughRange [mm]");
   histdEdxVTotalRangeBackwards->GetYaxis()->SetTitle("#frac{dE}{dx} [ADC/mm]");

   TCanvas *c8 = new TCanvas();
   histSiPIDADCMax->Draw("zcol");
   //cutSiB->Draw("same");
   //cutSiC->Draw("same");
   //cutSi13B->Draw("same");
   //cutSiBe->Draw("same");
   histSiPIDADCMax->GetXaxis()->SetTitle("ADC^{max}_{2} [ADC]");
   histSiPIDADCMax->GetYaxis()->SetTitle("ADC^{max}_{1} [ADC]");

   // Perine : Check Si PID with ATTPC PID conditions on p and d to confirm PID of protons
   TCanvas *c8_ProtonATTPC = new TCanvas();
   histSiPIDADCMax_ProtonATTPC->Draw("zcol");
   //cutSiB->Draw("same");
   //cutSiC->Draw("same");
   //cutSi13B->Draw("same");
   //cutSiBe->Draw("same");
   histSiPIDADCMax_ProtonATTPC->GetXaxis()->SetTitle("ADC^{max}_{2} [ADC]");
   histSiPIDADCMax_ProtonATTPC->GetYaxis()->SetTitle("ADC^{max}_{1} [ADC]");

   TCanvas *c8_DeuteronATTPC = new TCanvas();
   histSiPIDADCMax_DeuteronATTPC->Draw("zcol");
   //cutSiB->Draw("same");
   //cutSiC->Draw("same");
   //cutSi13B->Draw("same");
   //cutSiBe->Draw("same");
   histSiPIDADCMax_DeuteronATTPC->GetXaxis()->SetTitle("ADC^{max}_{2} [ADC]");
   histSiPIDADCMax_DeuteronATTPC->GetYaxis()->SetTitle("ADC^{max}_{1} [ADC]");

   TCanvas *c8_TritonATTPC = new TCanvas();
   histSiPIDADCMax_TritonATTPC->Draw("zcol");
   //cutSiB->Draw("same");
   //cutSiC->Draw("same");
   //cutSi13B->Draw("same");
   //cutSiBe->Draw("same");
   histSiPIDADCMax_TritonATTPC->GetXaxis()->SetTitle("ADC^{max}_{2} [ADC]");
   histSiPIDADCMax_TritonATTPC->GetYaxis()->SetTitle("ADC^{max}_{1} [ADC]");


   TCanvas *c11 = new TCanvas();
   histGaggPIDADCMax->Draw("colz");
   //cutGaggB->Draw("same");
   //cutGaggBe->Draw("same");
   //cutGaggLi->Draw("same");
   //cutGaggHe->Draw("same");
   histGaggPIDADCMax->GetXaxis()->SetTitle("#Sigma ADC^{max}_{Gagg} [ADC]");
   histGaggPIDADCMax->GetYaxis()->SetTitle("ADC^{max}_{2} [ADC]");



   // Saving histograms in a .root file ...
   TFile * Results = new TFile("kine_results_5109_5116.root","recreate");
   Results->cd();

   // dE Vs Total Range
   histdEdxVTotalRange->Write();
   histdEdxVTotalRangeBackwards->Write();
   cutPIDproton->Write("PIDCutProton");
   cutPIDproton_extension->Write("PIDCutProtonExtension");
   cutPIDdeuteron->Write("PIDCutDeuteron");

   // Kinematics
   histEstimatedKinEVThetaLABTotal->Write();
   histEstimatedKinEVThetaLAB_ProtonATTPC->Write();
   histEstimatedKinEVThetaLAB_ProtonATTPC_extended->Write();
   histEstimatedKinEVThetaLAB_ProtonATTPC_CarbonSi->Write();
   histEstimatedKinEVThetaLAB_DeuteronATTPC->Write();
   histEstimatedKinEVThetaLAB_CarbonSi->Write();
   histEstimatedKinEVThetaLAB_NitrogenSi->Write();
   kine_dp_gs->Write("kin_dp_gs");
   kine_dd_gs->Write("kin_dd_gs");
   kine_dd_gs_25MeVu->Write("kin_dd_gs_25MeVu");
   kine_dp_gs_25MeVu->Write("kin_dp_gs_25MeVu");

   // Excitation energy spectra
   histExdp->Write();
   histExdp_extended->Write();
   histExdp_CarbonSi->Write();
   histExdd->Write();

   // Angular distributions 
   histAngDist_elastic->Write();
   histAngDist_dp->Write();
   histAngDist_dp_CarbonSi->Write();

   // Si and Gagg PID
   histSiPIDADCMax->Write();
   histGaggPIDADCMax->Write();

   // Others ...
   // histThetaLABThetaLAB->Write();

   Results->Close();
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
