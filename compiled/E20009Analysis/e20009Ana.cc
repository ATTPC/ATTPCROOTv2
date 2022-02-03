#include "e20009Ana.h"

#include "AtFitter.h"
#include "AtGenfit.h"

#include <chrono>
#include <thread>
#include <iostream>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

int main(int argc, char *argv[])
{

   std::size_t firstEvt = 0;
   std::size_t lastEvt = 0;
   bool fInteractiveMode = 1;
   TString inputFileName = "";
   bool fitDirection = 0; // 0: Forward (d,d) - 1: Backwards (d,p)
   bool simulationConv = 0;
   
   //  Arguments
   if (argc == 7) {
      firstEvt = std::atoi(argv[1]);
      lastEvt = std::atoi(argv[2]);
      fInteractiveMode = std::atoi(argv[3]);
      inputFileName = argv[4];
      fitDirection = std::atoi(argv[5]);
      simulationConv = std::atoi(argv[6]);
      std::cout << cGREEN << " Processing file " << inputFileName << "\n";
      std::cout << " Processing events from : " << firstEvt << " to " << lastEvt << "\n";
      std::cout << " Fit direction : " << fitDirection << " (0: Forward (d,d) - 1: Backwards (d,p))\n ";
      std::cout << " Simulation ? "<<simulationConv<<"\n ";
      std::cout << " Interactive mode? : " << fInteractiveMode << cNORMAL << "\n";

      
      
   } else {
      std::cout << " Wrong number of arguments. Expecting 7: first_event last_event interactive_mode_bool "
                   "fileNameWithoutExtension fitDirection_bool simulation_convention."
                << "\n";
      return 0;
   }

   if (lastEvt < firstEvt) {
      std::cerr << " Error!: Inconsistent numbers for first/last event. Exiting... "
                << "\n";
      return 0;
   }

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();

   TString rootFileName = inputFileName + ".root";

   std::vector<TString> files;
   files.push_back(rootFileName);

   // Analysis parameters
   Float_t magneticField = 3.0; // T
   Float_t dMass = 2.0135532;
   Float_t pMass = 1.00727646;
   Float_t Be10Mass = 10.013533818;
   Float_t Be11Mass = 11.021657749;
   Int_t atomicNumber = 1;
   Int_t pPDGCode = 2212;
   Int_t dPDGCode = 1000010020;
   Int_t Be10PDGCode = 1000040100;
   Int_t Be11PDGCode = 1000040110;
   Int_t BeAtomicNumber = 4;

   Int_t particlePDG = 1000010020; // 1000010020;//2212;
   Float_t particleMass = 0;
   Int_t recoilPDG = 0;
   Float_t recoilMass = 0;

   // Q value calculation
   // Q-value calculation
   Double_t m_p = 1.007825 * 931.49401;
   Double_t m_d = 2.0135532 * 931.49401;
   Double_t m_Be10 = 10.013533818 * 931.49401;
   Double_t m_Be11 = 11.021657749 * 931.49401;
   Double_t m_beam = m_Be10;

   Double_t Ebeam_buff = 96.0; //(EnergyRecoil + EnergySca + ex_energy[iFile]);

   Double_t m_b;
   Double_t m_B;

   Float_t gasMediumDensity = 0.13129;
   
   TString elossFileName = "deuteron_D2_1bar.txt";

   switch (fitDirection) {
   case 1:
      std::cout << cGREEN << " Analyzing 10Be(d,p)11Be (PDG: 2212) " << cNORMAL << "\n";
      particleMass = pMass;
      recoilMass = Be11Mass;
      recoilPDG = 1000040110;
      m_b = m_p;
      m_B = m_Be11;
      particlePDG = 2212;
      elossFileName = "proton_D2_600torr.txt";
      break;
   case 0:
      std::cout << cGREEN << " Analyzing 10Be(d,d)10Be (PDG: 1000010020) " << cNORMAL << "\n";
      particleMass = dMass;
      recoilMass = Be10Mass;
      recoilPDG = 1000040100;
      m_b = m_d;
      m_B = m_Be10;
      particlePDG = 1000010020;
      elossFileName = "deuteron_D2_600torr.txt";
      break;
   }

   const Double_t M_Ener = particleMass * 931.49401 / 1000.0;

   // Histograms
   TH1F *angle = new TH1F("angle", "angle", 720, 0, 179);
   TH1F *hphi = new TH1F("hphi", "hphi", 1440, -359, 359);
   TH1F *phi_pattern = new TH1F("phi_pattern", "phi_pattern", 1440, -359, 359);
   TH2F *phi_phi_pattern = new TH2F("phi_phi_pattern", "phi_phi_pattern", 720, -359, 359, 720, -359, 359);
   TH1F *momentum = new TH1F("momentum", "momentum", 1000, 0, 2.0); // GeV
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);
   TH2F *pos_vs_momentum = new TH2F("pos_vs_momentum", "pos_vs_momentum", 200, 0, 200, 1000, 0, 2.0);
   TH2F *length_vs_momentum = new TH2F("length_vs_momentum", "length_vs_momentum", 200, 0, 200, 1000, 0, 2.0);
   TH2F *hits_vs_momentum = new TH2F("hits_vs_momentum", "hits_vs_momentum", 200, 0, 200, 1000, 0, 2.0);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 1000, 0, 100.0);
   TH2F *angle_vs_energy_pattern =
      new TH2F("angle_vs_energy_pattern", "angle_vs_energy_pattern", 720, 0, 179, 1000, 0, 100.0);
   TH1F *HQval = new TH1F("HQval", "HQval", 1000, -10, 10);

   TH2F *evtNum_vs_trkNum = new TH2F("evtNum_vs_trkNum", "evtNum_vs_trkNum", 1000, 0, 1000, 10, 0, 10);

   TH1F *eRes = new TH1F("eRes", "eRes", 100, -2.0, 2.0);

   TH1F *zpos_fit = new TH1F("zpos_fit", "zpos_fit", 200, -100, 100);

   // Paths
   TString dir = getenv("VMCWORKDIR");

   TString geoManFile = dir + "/geometry/ATTPC_D600torr_v2_geomanager.root";
   std::cout << " Geometry file : " << geoManFile.Data() << "\n";
   
   TString filePath;

   if(simulationConv)
     filePath = dir + "/macro/Simulation/ATTPC/10Be_dp/";
   else
      filePath = dir + "/macro/Unpack_HDF5/e20009/rootFiles/";

   TString fileName = "run_0108.root";

   TString fileNameWithPath = dir + filePath + fileName;

   TString eLossFilePath = dir + "/resources/energy_loss/";
   TString eLossFileNameWithPath = eLossFilePath + elossFileName;

   std::cout << " ELoss file " << eLossFileNameWithPath.Data() << "\n";

   FairRunAna *run = new FairRunAna();
   run->SetGeomFile(geoManFile.Data());
   TFile *file; // = new TFile(fileNameWithPath.Data(), "READ");

   // GENFIT geometry
   new TGeoManager("Geometry", "ATTPC geometry");
   TGeoManager::Import(geoManFile.Data());
   genfit::MaterialEffects::getInstance()->init(new genfit::TGeoMaterialInterface());
   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0., magneticField * 10.0)); //

   // event display
   genfit::EventDisplay *display;
   if (fInteractiveMode)
      display = genfit::EventDisplay::getInstance();

   AtFITTER::AtFitter *fFitter = new AtFITTER::AtGenfit(magneticField, 0.00001, 1000.0, eLossFileNameWithPath.Data(),gasMediumDensity);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetPDGCode(particlePDG);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetMass(particleMass);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetAtomicNumber(atomicNumber);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetNumFitPoints(1.0);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetVerbosityLevel(1);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetSimulationConvention(simulationConv);
   

   // Output file
   Float_t EFit;
   Float_t AFit;
   Float_t PhiFit;
   Float_t EPRA;
   Float_t APRA;
   Float_t PhiPRA;
   Float_t Ex;
   Float_t xiniFit;
   Float_t yiniFit;
   Float_t ziniFit;
   Float_t xiniPRA;
   Float_t yiniPRA;
   Float_t ziniPRA;
   Float_t pVal;
   Float_t IC;
   Float_t EFitXtr;
   Float_t ExXtr;
   Float_t xiniFitXtr;
   Float_t yiniFitXtr;
   Float_t ziniFitXtr;
   Float_t distXtr;
   Float_t trackLength;
   Float_t POCAXtr;
   Int_t trackID;
   Int_t ICMult;

   std::vector<Float_t> EFitVec;
   std::vector<Float_t> AFitVec;
   std::vector<Float_t> PhiFitVec;
   std::vector<Float_t> EPRAVec;
   std::vector<Float_t> APRAVec;
   std::vector<Float_t> PhiPRAVec;
   std::vector<Float_t> ExVec;
   std::vector<Float_t> xiniFitVec;
   std::vector<Float_t> yiniFitVec;
   std::vector<Float_t> ziniFitVec;
   std::vector<Float_t> xiniPRAVec;
   std::vector<Float_t> yiniPRAVec;
   std::vector<Float_t> ziniPRAVec;
   std::vector<Float_t> pValVec;
   std::vector<Float_t> ICVec;
   std::vector<Int_t> ICTimeVec;
   std::vector<Float_t> EFitXtrVec;
   std::vector<Float_t> ExXtrVec;
   std::vector<Float_t> xiniFitXtrVec;
   std::vector<Float_t> yiniFitXtrVec;
   std::vector<Float_t> ziniFitXtrVec;
   std::vector<Float_t> distXtrVec;
   std::vector<Float_t> trackLengthVec;
   std::vector<Float_t> POCAXtrVec;
   std::vector<Int_t> trackIDVec;
   std::vector<Float_t> fChi2Vec; 
   std::vector<Float_t> bChi2Vec;
   std::vector<Float_t> fNdfVec;
   std::vector<Float_t> bNdfVec;
   std::vector<Float_t> ICEVec;

   TString simFile;

   if(simulationConv)
     simFile = "_sim_";
   else
     simFile = "";
   
   TString outputFileName = "fit_analysis_"+ simFile + inputFileName;
   outputFileName += "_" + std::to_string(firstEvt) + "_" + std::to_string(lastEvt) + ".root";
   
   
   TFile *outputFile = new TFile(outputFileName.Data(), "RECREATE");
   TTree *outputTree = new TTree("outputTree", "OutputTree");
   outputTree->Branch("EFit", &EFit, "EFit/F");
   outputTree->Branch("AFit", &AFit, "AFit/F");
   outputTree->Branch("PhiFit", &PhiFit, "PhiFit/F");
   outputTree->Branch("EPRA", &EPRA, "EPRA/F");
   outputTree->Branch("APRA", &APRA, "APRA/F");
   outputTree->Branch("PhiPRA", &PhiPRA, "PhiPRA/F");
   outputTree->Branch("Ex", &Ex, "Ex/F");
   outputTree->Branch("ExXtr", &ExXtr, "ExXtr/F");
   outputTree->Branch("xiniFit", &xiniFit, "xiniFit/F");
   outputTree->Branch("yiniFit", &yiniFit, "yiniFit/F");
   outputTree->Branch("ziniFit", &ziniFit, "ziniFit/F");
   outputTree->Branch("xiniPRA", &xiniPRA, "xiniPRA/F");
   outputTree->Branch("yiniPRA", &yiniPRA, "yiniPRA/F");
   outputTree->Branch("ziniPRA", &ziniPRA, "ziniPRA/F");
   outputTree->Branch("EFitXtr", &EFitXtr, "EFitXtr/F");
   outputTree->Branch("xiniFitXtr", &xiniFitXtr, "xiniFitXtr/F");
   outputTree->Branch("yiniFitXtr", &yiniFitXtr, "yiniFitXtr/F");
   outputTree->Branch("ziniFitXtr", &ziniFitXtr, "ziniFitXtr/F");
   outputTree->Branch("distXtr", &distXtr, "distXtr/F");
   outputTree->Branch("pVal", &pVal, "pVal/F");
   outputTree->Branch("IC", &IC, "IC/F");
   outputTree->Branch("ICMult", &ICMult, "ICMult/I");
   outputTree->Branch("trackLength", &trackLength, "trackLength/F");
   outputTree->Branch("POCAXtr", &POCAXtr, "POCAXtr/F");
   outputTree->Branch("trackID", &trackID, "trackID/F");

   outputTree->Branch("EFitVec", &EFitVec);
   outputTree->Branch("AFitVec", &AFitVec);
   outputTree->Branch("PhiFitVec", &PhiFitVec);
   outputTree->Branch("EPRAVec", &EPRAVec);
   outputTree->Branch("APRAVec", &APRAVec);
   outputTree->Branch("PhiPRAVec", &PhiPRAVec);
   outputTree->Branch("ExVec", &ExVec);
   outputTree->Branch("ExXtrVec", &ExXtrVec);
   outputTree->Branch("xiniFitVec", &xiniFitVec);
   outputTree->Branch("yiniFitVec", &yiniFitVec);
   outputTree->Branch("ziniFitVec", &ziniFitVec);
   outputTree->Branch("xiniPRAVec", &xiniPRAVec);
   outputTree->Branch("yiniPRAVec", &yiniPRAVec);
   outputTree->Branch("ziniPRAVec", &ziniPRAVec);
   outputTree->Branch("EFitXtrVec", &EFitXtrVec);
   outputTree->Branch("xiniFitXtrVec", &xiniFitXtrVec);
   outputTree->Branch("yiniFitXtrVec", &yiniFitXtrVec);
   outputTree->Branch("ziniFitXtrVec", &ziniFitXtrVec);
   outputTree->Branch("distXtrVec", &distXtrVec);
   outputTree->Branch("pValVec", &pValVec);
   outputTree->Branch("ICVec", &ICVec);
   outputTree->Branch("ICTimeVec", &ICTimeVec);
   outputTree->Branch("trackLengthVec", &trackLengthVec);
   outputTree->Branch("POCAXtrVec", &POCAXtrVec);
   outputTree->Branch("trackIDVec", &trackIDVec);
   outputTree->Branch("fChi2Vec",&fChi2Vec); 
   outputTree->Branch("bChi2Vec",&bChi2Vec);
   outputTree->Branch("fNdfVec",&fNdfVec);
   outputTree->Branch("bNdfVec",&bNdfVec);
   outputTree->Branch("ICEVec", &ICEVec);

   for (auto iFile = 0; iFile < files.size(); ++iFile) {

     //fileNameWithPath = dir + filePath + files.at(iFile).Data();
     fileNameWithPath = filePath + files.at(iFile).Data();
     std::cout << " Opening File : " << fileNameWithPath.Data() << std::endl;

     file = new TFile(fileNameWithPath.Data(), "READ");

     Int_t nEvents = lastEvt - firstEvt;

     TTree *tree = (TTree *)file->Get("cbmsim");
     // Int_t nEvents = 100;//tree->GetEntries();
     std::cout << " Number of events : " << nEvents << std::endl;

     TTreeReader Reader1("cbmsim", file);
     TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
     TTreeReaderValue<TClonesArray> evArray(Reader1, "AtEventH");
     Reader1.SetEntriesRange(firstEvt, lastEvt);

     for (Int_t i = firstEvt; i < lastEvt; i++) {

        // std::chrono::seconds tickingBomb(10);
        // std::chrono::time_point<std::chrono::system_clock> end;
        // end = std::chrono::system_clock::now() + tickingBomb;

        // if(i%2==0)
        // continue;

        EFit = 0;
        EFitXtr = 0;
        AFit = 0;
        PhiFit = 0;
        EPRA = 0;
        APRA = 0;
        PhiPRA = 0;
        Ex = -100;
        ExXtr = -100;
        xiniFit = -100;
        yiniFit = -100;
        ziniFit = -1000;
        xiniFitXtr = -100;
        yiniFitXtr = -100;
        ziniFitXtr = -1000;
        xiniPRA = -100;
        yiniPRA = -100;
        ziniPRA = -1000;
        pVal = 0;
        IC = 0;
        ICMult = 0;
        trackLength = -1000.0;
        POCAXtr = -1000.0;

        EFitVec.clear();
        AFitVec.clear();
        PhiFitVec.clear();
        EPRAVec.clear();
        APRAVec.clear();
        PhiPRAVec.clear();
        ExVec.clear();
        xiniFitVec.clear();
        yiniFitVec.clear();
        ziniFitVec.clear();
        xiniPRAVec.clear();
        yiniPRAVec.clear();
        ziniPRAVec.clear();
        pValVec.clear();
        ICVec.clear();
        ICTimeVec.clear();
        EFitXtrVec.clear();
        ExXtrVec.clear();
        xiniFitXtrVec.clear();
        yiniFitXtrVec.clear();
        ziniFitXtrVec.clear();
        distXtrVec.clear();
        trackLengthVec.clear();
        POCAXtrVec.clear();
        trackIDVec.clear();
	fChi2Vec.clear(); 
        bChi2Vec.clear();
	fNdfVec.clear();
	bNdfVec.clear();
   ICEVec.clear();

   std::cout << cGREEN << " Event Number : " << i << cNORMAL << "\n";

   Reader1.Next();

   AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);
   AtEvent *event = (AtEvent *)evArray->At(0);

   if (patternEvent) {

      std::vector<AtPad> *auxPadArray = event->GetAuxPadArray();
      std::cout << " Number of auxiliary pads : " << auxPadArray->size() << "\n";

      std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
      std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";

      for (auto auxpad : *auxPadArray) {
         if (auxpad.GetAuxName().compare(std::string("IC_sca")) == 0) {
            std::cout << " Auxiliary pad name " << auxpad.GetAuxName() << "\n";
            Double_t *adc = auxpad.GetADC();
            ICMult = GetNPeaksHRS(&ICTimeVec, &ICVec, adc);
         }
         if (auxpad.GetAuxName().compare(std::string("IC")) == 0) {
            Double_t *adc = auxpad.GetADC();

            for (auto iadc = 0; iadc < 512; ++iadc)
               ICEVec.push_back(adc[iadc]);
         }
      }

      // ICVec.push_back(IC);

      evtNum_vs_trkNum->Fill(i, patternTrackCand.size());

      for (auto track : patternTrackCand) {

         std::cout << " Track " << track.GetTrackID() << " with " << track.GetHitClusterArray()->size() << " clusters "
                   << "\n";

         trackID = track.GetTrackID();
         trackIDVec.push_back(trackID);

         if (track.GetIsNoise() || track.GetHitClusterArray()->size() < 3) {
            std::cout << cRED << " Track is noise or has less than 3 clusters! " << cNORMAL << "\n";
            continue;
         }

         Double_t theta = track.GetGeoTheta();            // 180.0 * TMath::DegToRad() - track.GetGeoTheta();
         Double_t radius = track.GetGeoRadius() / 1000.0; // mm to m
         Double_t phi = track.GetGeoPhi();
         Double_t brho = magneticField * radius / TMath::Sin(theta); // Tm
         std::tuple<Double_t, Double_t> mom_ener = GetMomFromBrho(particleMass, atomicNumber, brho);
         angle_vs_energy_pattern->Fill(theta * TMath::RadToDeg(), std::get<1>(mom_ener) * 1000.0);
         phi_pattern->Fill(phi * TMath::RadToDeg());
         // phi_phi_pattern->Fill(phi * TMath::RadToDeg(), mom_res.Phi() * TMath::RadToDeg());
         EPRA = std::get<1>(mom_ener) * 1000.0;
         APRA = theta * TMath::RadToDeg();
         PhiPRA = phi * TMath::RadToDeg();

         std::cout << " Theta : " << theta * TMath::RadToDeg() << " Phi : " << phi * TMath::RadToDeg() << "\n";

         auto hitClusterArray = track.GetHitClusterArray();
         AtHitCluster iniCluster;
         Double_t zIniCal = 0;
         TVector3 iniPos;

         /*for (auto cluster : *hitClusterArray) {


 TVector3 pos = cluster.GetPosition();
 std::cout<<pos.X()<<"     "<<pos.Y()<<"   "<<pos.Z()<<"\n";

 }*/

         // Variable for convention (simulation comes reversed)
	      Double_t thetaConv;
	      if (simulationConv) {
		thetaConv = 180.0 - theta*TMath::RadToDeg();
	      } else {
		thetaConv = theta*TMath::RadToDeg();
	      }
	      
	      
              if (thetaConv < 90.0 ) {
                 iniCluster = hitClusterArray->back(); // NB: Use back because We do not reverse the cluster vector like in AtGenfit!
                 iniPos = iniCluster.GetPosition();
                 zIniCal = 1000.0 - iniPos.Z();
              } else if (thetaConv > 90.0 ) {
                 iniCluster = hitClusterArray->front();
                 iniPos = iniCluster.GetPosition();
                 zIniCal = iniPos.Z();
              }

              xiniPRA = iniPos.X();
              yiniPRA = iniPos.Y();
              ziniPRA = zIniCal;

	      //This is just to select distances
	      std::cout<<" Initial position : "<<xiniPRA<<" - "<<yiniPRA<<" - "<<ziniPRA<<"\n";

	      
              // Fit
              if (fitDirection == 0 && thetaConv > 90) // O is between 0 and 90 (simulation goes from 90 to 180) (d,d)
                 continue;
              else if (fitDirection == 1 && thetaConv < 90) // 1 is between 90 and 180 (simulation goes from 0 to 90) (d,p)
                 continue;

              // Skip border angles
              if (theta * TMath::RadToDeg() < 10 || theta * TMath::RadToDeg() > 170)
                 continue;

              // Skip tracks that are far from Z (to be checked against number of iterations for extrapolation)
              Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());

               std::cout<<cRED<<" Distance to Z "<<dist<<cNORMAL<<"\n";
              if (dist > 50.0)
                 continue;

              fFitter->Init();
              genfit::Track *fitTrack;

              try {
                 fitTrack = fFitter->FitTracks(&track);
              } catch (std::exception &e) {
                 std::cout << " Exception fitting track !" << e.what() << "\n";
                 continue;
              }

              if (fitTrack == nullptr)
                 continue;

              TVector3 pos_res;
              TVector3 mom_res;
              TMatrixDSym cov_res;
              // Double_t pVal = 0;
              Double_t bChi2 = 0, fChi2 = 0, bNdf = 0, fNdf = 0;
              Double_t distance = -100;
              Double_t POCA = 1E6;
              TVector3 mom_ext;
              TVector3 pos_ext;
              TVector3 mom_ext_buff;
              TVector3 pos_ext_buff;

              try {

                 if (fitTrack && fitTrack->hasKalmanFitStatus()) {

                    auto KalmanFitStatus = fitTrack->getKalmanFitStatus();
                    auto trackRep = fitTrack->getTrackRep(0); // Only one representation is sved for the moment.

                    if (KalmanFitStatus->isFitConverged(false)) {
		       // KalmanFitStatus->Print();
                       genfit::MeasuredStateOnPlane fitState = fitTrack->getFittedState();
		       fChi2 = KalmanFitStatus->getForwardChi2();
		       bChi2 = KalmanFitStatus->getBackwardChi2();
		       fNdf  = KalmanFitStatus->getForwardNdf();
		       bNdf  = KalmanFitStatus->getBackwardNdf();
		       // fitState.Print();
                       fitState.getPosMomCov(pos_res, mom_res, cov_res);
                       trackLength = KalmanFitStatus->getTrackLen();
                       pVal = KalmanFitStatus->getPVal();

                       // fKalmanFitter -> getChiSquNdf(gfTrack, trackRep, bChi2, fChi2, bNdf, fNdf);
                       Float_t stepXtr = -0.1;
                       Int_t minCnt = 0;
                       Int_t minCntExt = 0;

                       try {
                          for (auto iStep = 0; iStep < 40; ++iStep) {

                             trackRep->extrapolateBy(fitState, stepXtr * iStep);
                             mom_ext_buff = fitState.getMom();
                             pos_ext_buff = fitState.getPos();
                             double distance =
                                TMath::Sqrt(pos_ext_buff.X() * pos_ext_buff.X() + pos_ext_buff.Y() * pos_ext_buff.Y());
                             // if (fVerbosityLevel > 2){
                             /*std::cout << cYELLOW << " Extrapolation: Total Momentum : " << mom_ext_buff.Mag()
                                       << " - Position : " << pos_ext_buff.X() << "  " << pos_ext_buff.Y() << "  "
                                       << pos_ext_buff.Z() << " - distance : " << distance << cNORMAL << "\n";*/
                             //}

                             if (distance < POCA) {
                                POCA = distance;
                                POCAXtr = distance;
                                mom_ext = mom_ext_buff;
                                pos_ext = pos_ext_buff;
                                distXtr = iStep * stepXtr;
                                ++minCnt;
                                minCntExt = 0;
                             }
                             // Loop control
                             // if(minCntExt>20) //Break the loop if a new minimum is not found after several iterations
                             // break;

                             ++minCntExt;
                          }

                       } catch (genfit::Exception &e) {
                          mom_ext.SetXYZ(0, 0, 0);
                          pos_ext.SetXYZ(0, 0, 0);
                       }

                       // mom_res = mom_ext;
                       // pos_res = pos_ext;
                       xiniFitXtr = pos_ext.X();
                       yiniFitXtr = pos_ext.Y();
                       ziniFitXtr = pos_ext.Z();

                       std::cout << cYELLOW << " Extrapolation: Total Momentum : " << mom_ext.Mag()
                                 << " - Position : " << pos_ext.X() << "  " << pos_ext.Y() << "  " << pos_ext.Z()
                                 << " - POCA : " << POCA << cNORMAL << "\n";

                       // Building histograms
                       if (fInteractiveMode)
                          display->addEvent(fitTrack);

		       
                       Double_t thetaA = 0.0;
                       if (thetaConv > 90.0) {
                          thetaA = 180.0 * TMath::DegToRad() - mom_res.Theta();

                       } else {
                          thetaA = mom_res.Theta();
                       }

                       angle->Fill(thetaA * TMath::RadToDeg());
                       // std::cout<<" Angle "<<mom_res.Theta()<<"\n";
                       auto pos_radial = TMath::Sqrt(TMath::Power(pos_res.X(), 2) + TMath::Power(pos_res.Y(), 2));
                       momentum->Fill(mom_res.Mag());
                       angle_vs_momentum->Fill(thetaA * TMath::RadToDeg(), mom_res.Mag());
                       pos_vs_momentum->Fill(pos_res.Mag(), mom_res.Mag());
                       auto len = fitTrack->getTrackLen();
                       length_vs_momentum->Fill(len, mom_res.Mag());
                       auto numHits = fitTrack->getNumPoints();
                       hits_vs_momentum->Fill(numHits, mom_res.Mag());
                       Double_t E = TMath::Sqrt(TMath::Power(mom_res.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener;
                       angle_vs_energy->Fill(thetaA * TMath::RadToDeg(), E * 1000.0);
                       hphi->Fill(mom_res.Phi() * TMath::RadToDeg());

                       EFit = E * 1000.0;
                       AFit = thetaA * TMath::RadToDeg();
                       PhiFit = mom_res.Phi();

                       xiniFit = pos_res.X();
                       yiniFit = pos_res.Y();
                       ziniFit = pos_res.Z();

                       // Excitation energy
                       Double_t ex_energy_exp = kine_2b(m_Be10, m_d, m_b, m_B, Ebeam_buff, thetaA, E * 1000);
                       EFitXtr =
                          1000.0 * (TMath::Sqrt(TMath::Power(mom_ext.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener);
                       ExXtr = kine_2b(m_Be10, m_d, m_b, m_B, Ebeam_buff, thetaA, EFitXtr);

                       HQval->Fill(ex_energy_exp);

                       Ex = ex_energy_exp;
                    }
                 }
              } catch (std::exception &e) {
                 std::cout << " " << e.what() << "\n";
                 continue;
              }

              EPRAVec.push_back(EPRA);
              APRAVec.push_back(APRA);
              PhiPRAVec.push_back(PhiPRA);

              xiniPRAVec.push_back(xiniPRA);
              yiniPRAVec.push_back(yiniPRA);
              ziniPRAVec.push_back(ziniPRA);

              trackLengthVec.push_back(trackLength);
              pValVec.push_back(pVal);

              xiniFitXtrVec.push_back(xiniFitXtr);
              yiniFitXtrVec.push_back(yiniFitXtr);
              ziniFitXtrVec.push_back(ziniFitXtr);
              POCAXtrVec.push_back(POCAXtr);
              distXtrVec.push_back(distXtr);

              xiniFitVec.push_back(xiniFit);
              yiniFitVec.push_back(yiniFit);
              ziniFitVec.push_back(ziniFit);

              EFitVec.push_back(EFit);
              AFitVec.push_back(AFit);
              PhiFitVec.push_back(PhiFit);

              EFitXtrVec.push_back(EFitXtr);
              ExVec.push_back(Ex);
              ExXtrVec.push_back(ExXtr);

	      fChi2Vec.push_back(fChi2); 
              bChi2Vec.push_back(bChi2);
	      fNdfVec.push_back(fNdf);
	      bNdfVec.push_back(bNdf);

      } // track loop

           outputTree->Fill();

   } // if pattern event

      } // Event

   } // File

   outputFile->cd();
   // outputTree->Print();
   outputTree->Write();
   outputFile->Close();

   // Adding kinematic lines
   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "../Be10pp_el.txt";
   std::ifstream *kineStr = new std::ifstream(fileKine.Data());
   Int_t numKin = 0;

   if (!kineStr->fail()) {
      while (!kineStr->eof()) {
         *kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec = new TGraph(numKin, ThetaLabRec, EnerLabRec);



   fileKine = "../Be10pp_el_9AMeV.txt";
   std::ifstream *kineStr5 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr5->fail()) {
      while (!kineStr5->eof()) {
         *kineStr5 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec_9AMeV = new TGraph(numKin, ThetaLabRec, EnerLabRec);




   
   fileKine = "../Be10pp_in_2+1.txt";
   std::ifstream *kineStr2 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr2->fail()) {
      while (!kineStr2->eof()) {
         *kineStr2 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr2->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec_in = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   fileKine = "../Be10dp_gs.txt";
   std::ifstream *kineStr3 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr3->fail()) {
      while (!kineStr3->eof()) {
         *kineStr3 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr3->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec_dp = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   fileKine = "../Be10dp_first.txt";
   std::ifstream *kineStr4 = new std::ifstream(fileKine.Data());
   numKin = 0;

   if (!kineStr4->fail()) {
      while (!kineStr4->eof()) {
         *kineStr4 >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         numKin++;
      }
   } else if (kineStr4->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec_dp_first = new TGraph(numKin, ThetaLabRec, EnerLabRec);

   if (fInteractiveMode) {

      TCanvas *c1 = new TCanvas();
      c1->Divide(2, 2);
      c1->Draw();
      c1->cd(1);
      angle->Draw();
      c1->cd(2);
      momentum->Draw();
      c1->cd(3);
      angle_vs_momentum->Draw();
      c1->cd(4);
      pos_vs_momentum->Draw();

      TCanvas *cKine = new TCanvas();
      cKine->Divide(1, 2);
      cKine->cd(1);
      cKine->Draw();
      angle_vs_energy->SetMarkerStyle(20);
      angle_vs_energy->SetMarkerSize(0.5);
      angle_vs_energy->Draw();
      Kine_AngRec_EnerRec->SetLineWidth(1);
      Kine_AngRec_EnerRec->SetLineColor(kRed);
      Kine_AngRec_EnerRec->Draw("SAME");
      Kine_AngRec_EnerRec_9AMeV->SetLineWidth(1);
      Kine_AngRec_EnerRec_9AMeV->SetLineColor(kRed+1);
      Kine_AngRec_EnerRec_9AMeV->Draw("SAME");
      Kine_AngRec_EnerRec_in->SetLineWidth(1);
      Kine_AngRec_EnerRec_in->SetLineColor(kBlue);
      Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
      Kine_AngRec_EnerRec_dp->SetLineWidth(1);
      Kine_AngRec_EnerRec_dp->SetLineColor(kGreen);
      Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
      Kine_AngRec_EnerRec_dp_first->SetLineWidth(1);
      Kine_AngRec_EnerRec_dp_first->SetLineColor(kViolet);
      Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");
      cKine->cd(2);
      angle_vs_energy_pattern->Draw();
      Kine_AngRec_EnerRec->Draw("SAME");
      Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
      Kine_AngRec_EnerRec_dp->Draw("ZCOL SAME");
      Kine_AngRec_EnerRec_dp_first->Draw("ZCOL SAME");

      TCanvas *c2 = new TCanvas();
      c2->Divide(2, 2);
      c2->Draw();
      c2->cd(1);
      length_vs_momentum->Draw();
      c2->cd(2);
      hits_vs_momentum->Draw();

      c2->cd(3);
      hphi->Draw();

      c2->cd(4);
      phi_pattern->Draw();

      TCanvas *c3 = new TCanvas();
      c3->Divide(2, 2);
      c3->Draw();
      c3->cd(1);
      phi_phi_pattern->Draw();
      c3->cd(2);
      HQval->Draw();
      // c3->cd(3);
      // evtNum_vs_trkNum->Draw("zcol");

      // open event display
      display->open();

   } // Interactive mode

   return 0;
}

std::tuple<Double_t, Double_t> GetMomFromBrho(Double_t M, Double_t Z, Double_t brho)
{

   const Double_t M_Ener = M * 931.49401 / 1000.0;
   Double_t p = brho * Z * (2.99792458 / 10.0); // In GeV
   Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
   std::cout << " Brho : " << brho << " - p : " << p << " - E : " << E << "\n";
   return std::make_tuple(p, E);
}

double kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject)
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

   // THcm = theta_cm*TMath::RadToDeg();
   return Ex;
}

Double_t GetNPeaksHRS(std::vector<Int_t> *timeMax, std::vector<Float_t> *adcMax, double *adc_test)
{
   TSpectrum *s = new TSpectrum();
   Double_t dest[512];
   // Int_t nfound = s->Search(h1_test,2," ",0.25);//2 and 0.15
   Int_t nfound;
   nfound = s->SearchHighRes(adc_test, dest, 512, 2, 2, kFALSE, 1, kFALSE, 1);
   // nfound = s->SearchHighRes(adc_test, dest, 512, 2, 2, kTRUE, 3, kTRUE, 3);

   for (auto iPeak = 0; iPeak < nfound; ++iPeak) {

      Int_t time = (Int_t)(ceil((s->GetPositionX())[iPeak]));
      timeMax->push_back(time);
      adcMax->push_back(adc_test[time]);
   }

   delete s;
   return nfound;
}

double GetMaximum(double *adc)
{

   double max = 0;

   for (int indTB = 0; indTB < 512; ++indTB) {
      // std::cout<<" TB "<<indTB<<" adc "<<adc[indTB]<<"\n";
      if (adc[indTB] > max)
         max = adc[indTB];
   }

   return max;
}
