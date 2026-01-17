TGraph* ReadKinematics(TString kineFile);
Double_t omega(Double_t x, Double_t y, Double_t z);
std::tuple<double, double> kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject);

void kine_ana_transfer_vertex(int runNum)
{
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
   double density = 1.3321e-3; // 470Torr
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

   TFile *cutPIDFilep = new TFile("./cutFiles/PID_proton.root", "READ"); 
   TCutG *cutPIDproton = (TCutG *)cutPIDFilep->Get("cutPIDproton");
   cutPIDFilep->Close();

   TFile *cutPIDFile = new TFile("./cutFiles/PID_deuteron.root", "READ"); 
   TCutG *cutPIDdeuteron = (TCutG *)cutPIDFile->Get("cutPIDdeuteron");
   cutPIDFile->Close();

   TFile *cutPIDFile2 = new TFile("./cutFiles/PID_3He.root", "READ"); 
   TCutG *cutPID3He = (TCutG *)cutPIDFile2->Get("cutPID3He");
   cutPIDFile2->Close();
   
   TFile *cutSiPIDFile = new TFile("./cutFiles/SiPID.root", "READ");
   TCutG *cutSiB = (TCutG *)cutSiPIDFile->Get("cutSiPIDB");
   TCutG *cutSiC = (TCutG *)cutSiPIDFile->Get("cutSiPIDC");
   TCutG *cutSi13B = (TCutG *)cutSiPIDFile->Get("cutSiPID13B");
   TCutG *cutSiBe = (TCutG *)cutSiPIDFile->Get("cutSiPIDBe");
   cutSiPIDFile->Close();

   TFile *cutGaggPIDFile = new TFile("./cutFiles/GAGGPID.root", "READ");
   TCutG *cutGaggB = (TCutG *)cutGaggPIDFile->Get("cutGAGGPIDB");
   TCutG *cutGaggBe = (TCutG *)cutGaggPIDFile->Get("cutGAGGPIDBe");
   TCutG *cutGaggLi = (TCutG *)cutGaggPIDFile->Get("cutGAGGPIDLi");
   TCutG *cutGaggHe = (TCutG *)cutGaggPIDFile->Get("cutGAGGPIDHe");
   cutGaggPIDFile->Close();


   // Kinematic curve
   TGraph* kinecurve_3HeGS = new TGraph("ang_lab_cm_3HeGS.txt","%lg %*s %lg");
   kinecurve_3HeGS->SetLineWidth(2);
   kinecurve_3HeGS->SetLineColor(kRed);
   TGraph* ang_lab_cm_3HeGS = new TGraph("ang_lab_cm_3HeGS.txt","%lg %*s %lg");// 5 deg pitch in theta_cm
   ang_lab_cm_3HeGS->SetMarkerStyle(8);
   ang_lab_cm_3HeGS->SetMarkerSize(1);
   TLegend *legend = new TLegend(0.6,0.2,0.85,0.5);
   legend->AddEntry(kinecurve_3HeGS,"3He G.S.","l");
   legend->AddEntry(ang_lab_cm_3HeGS,"5 deg pitch in #theta_{cm}","p");
   legend->SetFillColor(0);
   


   // Macro test
   //   std::vector runNums = {3001, 3002, 3003,3004,3005};


      // Open the digitalization file and get the TTree.
   TString unpackFileName = TString::Format("/data/ATTPCROOTv2_results/E581/UnpackerOutput/run_%04d_reUnpack.root", runNum);
   TFile *unpackFile = new TFile(unpackFileName, "READ");
   
   if(!unpackFile){
	cout<<"can not find file :"<<runNum<<endl;
   }
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

   // Open the GAGG file of this run.
   /*TString gaggFileName = TString::Format("/data/sustech/user/public/frib/frib-decode/data/hit%04d.root", runNum);
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
   gaggTree->SetBranchAddress("ADC_max_g2", gaggADCMax2);*/

   // Loop over events.
   Int_t track_num;
   Double_t kineE_p[10];
   Double_t kineE_d[10];
   Double_t kineE_He3[10];

   Double_t thetaLab[10];
   Double_t phiLab[10];
   Double_t range[10];
   Double_t dedx[10];

   Double_t first_x[10];
   Double_t first_y[10];
   Double_t first_z[10];

   Double_t last_x[10];
   Double_t last_y[10];
   Double_t last_z[10];

   bool punch_through[10];

   Double_t SiFe1;
   Double_t SiFe2;  
   Int_t SiFn1;
   Int_t SiFn2; 
   Double_t GAGGe1;
   Double_t GAGGid1;
   
   Double_t vertex_x, vertex_y, vertex_z;
   
   
   TString output_filename = TString::Format("./vertex_data/vertex_%d_test.root", runNum);
   TFile *ipf = new TFile(output_filename.Data(), "RECREATE");
   TTree *tree_new = new TTree("tree","several variables from ATTPC unpacker file");

   tree_new->Branch("track_num", &track_num, "track_num/I");

   tree_new->Branch("kineE_p", kineE_p, "kineE_p[track_num]/D");
   tree_new->Branch("kineE_d", kineE_d, "kineE_d[track_num]/D");
   tree_new->Branch("kineE_He3", kineE_He3, "kineE_He3[track_num]/D");

   tree_new->Branch("thetaLab", thetaLab, "thetaLab[track_num]/D");
   tree_new->Branch("phiLab", phiLab, "phiLab[track_num]/D");
   tree_new->Branch("range", range, "range[track_num]/D");
   tree_new->Branch("dedx", dedx, "dedx[track_num]/D");
   
   tree_new->Branch("first_x", first_x, "first_x[track_num]/D");
   tree_new->Branch("first_y", first_y, "first_y[track_num]/D");
   tree_new->Branch("first_z", first_z, "first_z[track_num]/D");

   tree_new->Branch("last_x", last_x, "last_x[track_num]/D");
   tree_new->Branch("last_y", last_y, "last_y[track_num]/D");
   tree_new->Branch("last_z", last_z, "last_z[track_num]/D");
	
   tree_new->Branch("vertex_x", &vertex_x, "vertex_x/D");
   tree_new->Branch("vertex_y", &vertex_y, "vertex_y/D");
   tree_new->Branch("vertex_z", &vertex_z, "vertex_z/D");

   tree_new->Branch("punch_through", punch_through, "punch_through[track_num]/b");


   tree_new->Branch("SiFe1", &SiFe1, "SiFe1/D");
   tree_new->Branch("SiFe2", &SiFe2, "SiFe2/D");
   tree_new->Branch("SiFn1", &SiFn1, "SiFn1/I");
   tree_new->Branch("SiFn2", &SiFn2, "SiFn2/I");

   tree_new->Branch("GAGGe1", &GAGGe1, "GAGGe1/D");
   tree_new->Branch("GAGGid1", &GAGGid1, "GAGGid1/D");

   TH2D *pad_plane = new TH2D("pad_plane", "pad_plane",200, -250, 250, 200, -250, 250);
   
   for (int iEvent = 0; iEvent < nUnpackEvents; iEvent++) {

      for(int i = 0; i < 10; i++){
         thetaLab[i] = -1;
         phiLab[i] = -1;
         range[i] = -1;
         dedx[i] = -1;
	      kineE_p[i] = -1;
	      kineE_d[i] = -1;
	      kineE_He3[i] = -1;

         first_x[i] = -999;
         first_y[i] = -999;
         first_z[i] = -999;

         last_x[i] = -999;
         last_y[i] = -999;
         last_z[i] = -999;

         punch_through[i] = 0;
      }
      track_num = 0;
      
      SiFe1 = -1;
      SiFe2 = -1;
      SiFn1 = -1;
      SiFn2 = -1;

      GAGGe1 = -1;
      GAGGid1 = -1;
      
      vertex_x = -999;
	vertex_y = -999;
	vertex_z = -999;
      unpackReader.Next();


      // Check the Si data first.
      AtSiEvent *siEvent = (AtSiEvent *)siArray->At(0);

      Int_t multiplicityFront1 = siEvent->GetMultiplicityFront1();
      Int_t multiplicityFront2 = siEvent->GetMultiplicityFront2();


      //if (multiplicityFront1 != 1 || multiplicityFront2 != 1) continue;

      Double_t maxADCFront1 = siEvent->GetADCMaxFront1(0);
      Double_t maxADCFront2 = siEvent->GetADCMaxFront2(0);

      Double_t EFront1 = siEvent->GetEFront1(0);
      Double_t EFront2 = siEvent->GetEFront2(0);

      SiFe1 = maxADCFront1;
      SiFe2 = maxADCFront2;

      SiFn1 = multiplicityFront1;
      SiFn2 = multiplicityFront2;



         // Check the GAGG data directly from the GAGG decoder.
         /*Double_t totalMaxADCGagg1{};
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

         //if (!cutGaggC->IsInside(totalMaxADCGagg, maxADCFront2)) continue;*/

         // Check the GAGG data
      AtGaggEvent *gaggEvent = (AtGaggEvent *)gaggArray->At(0);

      Int_t multiGagg1 = gaggEvent->GetMultiplicity1();
      Int_t multiGagg2 = gaggEvent->GetMultiplicity2();

      //std::cout << " Multiplicity GAGG1: " << multiGagg1 <<std::endl;

      //if (multiGagg1 != 1) continue;
      Double_t maxADCGagg1, maxADCGagg2;
      Double_t gaggADC1{-1}, gaggADC2{-1};
      Double_t totalMaxADCGagg{};
      for(int imul = 0; imul < multiGagg1; imul++) {
         maxADCGagg1 = gaggEvent->GetADCMax1(imul);
         gaggADC1 += maxADCGagg1;
         totalMaxADCGagg += maxADCGagg1;
         //std::cout << " ADC entry = " << maxADCGagg1 << std::endl;
      }
      for(int imul = 0; imul < multiGagg2; imul++) {
         maxADCGagg2 = gaggEvent->GetADCMax2(imul);
         gaggADC2 += maxADCGagg2;
         totalMaxADCGagg += maxADCGagg2;
      }



         // First, we obtain some rough kinematics just by using the AtPatternEvent.
      AtPatternEvent *patternEvent = (AtPatternEvent *)patternArray->At(0);
      if (!patternEvent){
	 	track_num = 0;
		tree_new->Fill();
		continue;
	}

      // We want to focus on events with 2 or less tracks for now.
      auto &tracks = patternEvent->GetTrackCand();
      
       if (tracks.size() > 2){
	//track_num = 0; 
	//track_num = tracks.size();
	//tree_new->Fill();	
	// continue;
		auto *findvertex = new AtFindVertex();
		findvertex->FindVertex(tracks, 3);
		auto tracks_from_vertex = findvertex->GetTracksVertex();
		if(tracks_from_vertex.size() > 0){
		auto beam_vertex = tracks_from_vertex[0].vertex;
		vertex_x = beam_vertex.X();
		vertex_y = beam_vertex.Y();
		vertex_z = beam_vertex.Z();
//		cout<<"beam veertex  X: "<<beam_vertex.X()<<" Y: "<<beam_vertex.Y()<<" Z: "<<beam_vertex.Z()<<endl;
		}
	}
         //if (tracks.size() > 1) continue;
      if (tracks.size()==2) nEventsWith2Tracks++;
      int trackIndex = 0;
      double thetaLABArray[2] = {0.0,0.0};
      double estimatedKineEArray[2] = {0.0,0.0};
      bool isHe3Array[2] = {false,false};
         // Iterate over AtTracks and extract their kinematics.



      for (auto &track: tracks){
         bool isPunchThrough = punchThroughChecker.IsPunchThrough(&track);
         //if (isPunchThrough) continue;


         auto *pattern = track.GetPattern();

         auto firstPoint = track.GetFirstPoint();
         auto lastPoint = track.GetLastPoint();

         //cut penetrate particles
         //if (lastPoint.Z()>900) continue;
         //std::cout<<lastPoint.Z()<<std::endl;
         first_x[track_num] = firstPoint.X();
         first_y[track_num] = firstPoint.Y();
         first_z[track_num] = firstPoint.Z();

         last_x[track_num] = lastPoint.X();
         last_y[track_num] = lastPoint.Y();
         last_z[track_num] = lastPoint.Z();

         double roughRangeEstimation = pattern->DistanceAlongPattern(lastPoint, firstPoint);

         double trackThetaLAB = track.GetGeoTheta() * 180 / TMath::Pi();
         double trackPhi = track.GetGeoPhi() * 180 / TMath::Pi();

         double smallPadCharge{};
         double bigPadCharge{};
         auto braggCurvePairs = track.GetBraggCurveValues();
         auto &hits = track.GetHitArray();
         double rangeInSmallPads{};
         
         for (auto &hit: hits){
            int padNum = hit->GetPadNum();
            int sizeID = map->GetPadSize(padNum);
	     
	    auto ipoint = hit->GetPosition();
	    pad_plane->Fill(ipoint.X(), ipoint.Y());
           
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
           //if (cutPID3He->IsInside(roughRangeEstimation, dEdx)) {
         if(roughRangeEstimation > 0 && roughRangeEstimation < 5000){ 
	   while (eLossModelC3D8_3He->GetRange(estimatedKinE) < roughRangeEstimation)
               estimatedKinE += 0.01;

	      kineE_He3[track_num] = estimatedKinE;

	     
	    //else if (cutPIDdeuteron->IsInside(roughRangeEstimation, dEdx)) {
            estimatedKinE = 0.1;
	    while (eLossModelC3D8_d->GetRange(estimatedKinE) < roughRangeEstimation)
               estimatedKinE += 0.01;
	      
	      kineE_d[track_num] = estimatedKinE;

	     
	    //else if (cutPIDproton->IsInside(roughRangeEstimation, dEdx)) {
            estimatedKinE = 0.1;
	    while (eLossModelC3D8_p->GetRange(estimatedKinE) < roughRangeEstimation)
               estimatedKinE += 0.01;
         
	       kineE_p[track_num] = estimatedKinE;
	    
	 }
	 else {
	         estimatedKinE = 0;
         }

         punch_through[track_num] = isPunchThrough;
         thetaLab[track_num] = trackThetaLAB;
         phiLab[track_num] = trackPhi;
         range[track_num] = roughRangeEstimation;
         dedx[track_num] = dEdx;

         track_num += 1;

      }
      tree_new->Fill();
      if(iEvent % 10000 == 0)cout<<"processing : "<<iEvent<<" / "<<nUnpackEvents<<endl;
   }
   
   ipf->cd();
   pad_plane->Write();
   tree_new->Write();
   ipf->Close();
   //      std::cout << "Number of 2 tracks events in run" << runNum << ":" << nEventsWith2Tracks << std::endl;
   //      std::cout << "Number of events with 3He in run" << runNum << ":" << nEventsWith3He << std::endl;
   // Close files.
   unpackFile->Close();
   //gaggFile->Close();
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
	//         *kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >>
	//                     ThetaLabSca[numKin] >> EnerLabSca[numKin];
         *kineStr >> ThetaLabRec[numKin] >> EnerLabRec[numKin];
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
