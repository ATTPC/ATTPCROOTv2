#include "e20009Ana_v1.h"

#include "AtFitter.h"
#include "AtGenfit.h"

#include <chrono>
#include <thread>
#include <iostream>

//#include <TMath.h>
//#include <TCanvas.h>
//#include <TFile.h>
//#include <TTree.h>
//#include <TH1I.h>
//#include <TGraph.h>
//#include <TF1.h>
//#include <TSpectrum.h>
//#include <fstream>
//#include <TStyle.h>

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

#define m_p 1.007825031898
#define m_d 2.014101777844
#define m_10B 10.012936862
#define m_11B 11.009305166
#define Conv_AMU_to_MeV 931.49401

#define magneticField 3.0 // Tesla
#define atomicNumber 1
#define BAtomicNumber 5

#define pPDGCode 2212;
#define dPDGCode 1000010020
#define B10PDGCode 1000050100
//#define Be10 1000040100
#define B11PDGCode 1000050110
//#define Be11 1000040110
#define particlePDG 1000010020

#define m_beam M_10B //select your beam particle here
#define Ebeam_buff 143.94 //MeV
/////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]){

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
                   "fileNameWithoutExtension fitDirection_bool simulation_convention."<< "\n";
      return 0;
   }

   if (lastEvt < firstEvt) {
      std::cerr << " Error!: Inconsistent numbers for first/last event. Exiting... "<< "\n";
      return 0;
   }

   // -----   Timer   --------------------------------------------------------
   TStopwatch timer;
   timer.Start();

   TString rootFileName = inputFileName + ".root";

   std::vector<TString> files;
   files.push_back(rootFileName);
   
   Float_t particleMass = 0;
   Int_t recoilPDG = 0;
   Float_t recoilMass = 0;

   // Q-value calculation
   Double_t M_P = m_p * Conv_AMU_to_MeV;
   Double_t M_D = m_d * Conv_AMU_to_MeV;
   Double_t M_10B = m_10B * Conv_AMU_to_MeV;
   Double_t M_11B = m_11B * Conv_AMU_to_MeV;
   
   Double_t m_b;
   Double_t m_B;

   TString elossFileName = "deuteron_D2_1bar.txt";

   switch (fitDirection) {
   case 1:
      std::cout << cGREEN << " Analyzing 10B(d,p)11B (PDG: 2212) " << cNORMAL << "\n";
      particleMass = m_p;
      recoilMass = m_11B;
      recoilPDG = 1000040110;
      m_b = M_P;
      m_B = M_11B;
      elossFileName = "proton_D2_600torr.txt";
      break;
   case 0:
      std::cout << cGREEN << " Analyzing 10B(d,d)10B (PDG: 1000010020) " << cNORMAL << "\n";
      particleMass = m_d;
      recoilMass = m_10B;
      recoilPDG = 1000040100;
      m_b = M_D;
      m_B = M_10B;
      elossFileName = "deuteron_D2_600torr.txt";
      break;
   }

   const Double_t M_Ener = particleMass * Conv_AMU_to_MeV / 1000.0;

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

   TString geoManFile = dir + "/geometry/ATTPC_D600torr_NR_v1_geomanager.root";
   std::cout << " Geometry file : " << geoManFile.Data() << "\n";
   
   TString filePath;

   if(simulationConv){
     //filePath = dir + "/macro/Simulation/ATTPC/10B_dp/";
     filePath = dir + "/mnt/analysis/e20009/e20009_Nabin/analysis/simulation/rootfiles/";//change it
   }else{ 
     //filePath = "/mnt/analysis/e20009/root_files/";
     filePath = "/mnt/analysis/e20009/e20009_Nabin/analysis/unpacker/DV0p9372/";//change it
     //filePath = "/mnt/analysis/e20009/e20009_Nabin/analysis/unpacker/Good_files_10252021/";//change it
   }
   
   TString fileName = "run_0344DV_0.9372.root";//run_0280DV_1.00.root
   //TString fileName = "Sim_out_digi_B10dp_gs_2500ev_09012021.root";
   
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

   AtFITTER::AtFitter *fFitter = new AtFITTER::AtGenfit(magneticField, 0.00001, 1000.0, eLossFileNameWithPath.Data());
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
   Double_t IC_sca;
   Double_t DB_beam;
   Double_t trigger_live;
   Double_t trigger_free;
   Double_t mesh;
   Double_t mesh_MCA;
   
   Float_t EFitXtr;
   Float_t ExXtr;
   Float_t xiniFitXtr;
   Float_t yiniFitXtr;
   Float_t ziniFitXtr;
   Float_t distXtr;
   Float_t trackLength;
   Float_t POCAXtr;
   Int_t trackID;

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
   std::vector<Double_t> IC_sca_Vec;
   std::vector<Double_t> DB_beam_Vec;
   std::vector<Double_t> trigger_live_Vec;
   std::vector<Double_t> trigger_free_Vec;
   std::vector<Double_t> mesh_Vec;
   std::vector<Double_t> mesh_MCA_Vec;
   
   std::vector<Float_t> EFitXtrVec;
   std::vector<Float_t> ExXtrVec;
   std::vector<Float_t> xiniFitXtrVec;
   std::vector<Float_t> yiniFitXtrVec;
   std::vector<Float_t> ziniFitXtrVec;
   std::vector<Float_t> distXtrVec;
   std::vector<Float_t> trackLengthVec;
   std::vector<Float_t> POCAXtrVec;
   std::vector<Int_t> trackIDVec;

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
   outputTree->Branch("IC_sca", &IC_sca, "IC_sca/F");
   outputTree->Branch("DB_beam", &DB_beam, "DB_beam/F");
   outputTree->Branch("trigger_live", &trigger_live, "trigger_live/F");
   outputTree->Branch("trigger_free", &trigger_free, "trigger_free/F");
   outputTree->Branch("mesh", &mesh, "mesh/F");
   outputTree->Branch("mesh_MCA", &mesh_MCA, "mesh_MCA/F");
   
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
   outputTree->Branch("IC_sca_Vec", &IC_sca_Vec);
   outputTree->Branch("DB_beam_Vec", &DB_beam_Vec);
   outputTree->Branch("trigger_live_Vec", &trigger_live_Vec);
   outputTree->Branch("ICVec", &ICVec);
   outputTree->Branch("mesh_Vec", &mesh_Vec);
   outputTree->Branch("mesh_MCA_Vec", &mesh_MCA_Vec);
   
   outputTree->Branch("trackLengthVec", &trackLengthVec);
   outputTree->Branch("POCAXtrVec", &POCAXtrVec);
   outputTree->Branch("trackIDVec", &trackIDVec);

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

        EFit = std::sqrt(-1);
        EFitXtr = std::sqrt(-1);
        AFit = std::sqrt(-1);
        PhiFit = std::sqrt(-1);
        EPRA = std::sqrt(-1);
        APRA = std::sqrt(-1);
        PhiPRA = std::sqrt(-1);
        Ex = std::sqrt(-1);
        ExXtr = std::sqrt(-1);
        xiniFit = std::sqrt(-1);
        yiniFit = std::sqrt(-1);
        ziniFit = std::sqrt(-1);
        xiniFitXtr = std::sqrt(-1);
        yiniFitXtr = std::sqrt(-1);
        ziniFitXtr = std::sqrt(-1);
        xiniPRA = std::sqrt(-1);
        yiniPRA = std::sqrt(-1);
        ziniPRA = std::sqrt(-1);
        pVal = std::sqrt(-1);
	
        IC = std::sqrt(-1);
	IC_sca = std::sqrt(-1);
	DB_beam = std::sqrt(-1);
	trigger_live = std::sqrt(-1);
	trigger_free = std::sqrt(-1);
	mesh = std::sqrt(-1);
	mesh_MCA = std::sqrt(-1);
	
        trackLength = std::sqrt(-1);
        POCAXtr = std::sqrt(-1);

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
	IC_sca_Vec.clear();
	DB_beam_Vec.clear();
	trigger_live_Vec.clear();
	trigger_free_Vec.clear();
	mesh_Vec.clear();
	mesh_MCA_Vec.clear();
	
	
        EFitXtrVec.clear();
        ExXtrVec.clear();
        xiniFitXtrVec.clear();
        yiniFitXtrVec.clear();
        ziniFitXtrVec.clear();
        distXtrVec.clear();
        trackLengthVec.clear();
        POCAXtrVec.clear();
        trackIDVec.clear();

        std::cout << cGREEN << " Event Number : " << i << cNORMAL << "\n";

        Reader1.Next();

        AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);
        AtEvent *event = (AtEvent *)evArray->At(0);

        if (patternEvent) {

           std::vector<AtPad> *auxPadArray = event->GetAuxPadArray();
           std::cout << " Number of auxiliary pads : " << auxPadArray->size() << "\n";

           std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
           std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";
	   //---------------------------
           for (auto auxpad : *auxPadArray) {
              if (auxpad.GetAuxName().compare(std::string("IC")) == 0) {
                 std::cout << " Auxiliary pad name " << auxpad.GetAuxName() << "\n";
                 Double_t *adc = auxpad.GetADC();
                 IC = GetMaximum(adc);
              }
           }
           ICVec.push_back(IC);
	   //---------------------------
	   for (auto auxpad : *auxPadArray) {
              if (auxpad.GetAuxName().compare(std::string("IC_sca")) == 0) {
                 std::cout << " Auxiliary pad name " << auxpad.GetAuxName() << "\n";
                 Double_t *adc_a = auxpad.GetADC();
                 //IC_sca = GetMaximum(adc_a);
		 //	 IC_sca = GetNPeaksHRS(adc_a);
		 IC_sca = GetNPeaksHRS(adc_a);
		 //IC_sca = adc_a;
              }
           }
           IC_sca_Vec.push_back(IC_sca);
	   //---------------------------
	   for (auto auxpad : *auxPadArray) {
              if (auxpad.GetAuxName().compare(std::string("DB_beam")) == 0) {
                 std::cout << " Auxiliary pad name " << auxpad.GetAuxName() << "\n";
                 Double_t *adc_b = auxpad.GetADC();
                 DB_beam = GetMaximum(adc_b);
		 //DB_beam = adc_b;
              }
           }
           DB_beam_Vec.push_back(DB_beam);
	   //---------------------------
	   for (auto auxpad : *auxPadArray) {
              if (auxpad.GetAuxName().compare(std::string("trigger_live")) == 0) {
                 std::cout << " Auxiliary pad name " << auxpad.GetAuxName() << "\n";
                 Double_t *adc_c = auxpad.GetADC();
		 trigger_live = GetMaximum(adc_c);
		 //trigger_live = adc_c;
              }
           }
           trigger_live_Vec.push_back(trigger_live);
	   //---------------------------
	   for (auto auxpad : *auxPadArray) {
              if (auxpad.GetAuxName().compare(std::string("trigger_free")) == 0) {
                 std::cout << " Auxiliary pad name " << auxpad.GetAuxName() << "\n";
                 Double_t *adc_d = auxpad.GetADC();         
		 trigger_free = GetMaximum(adc_d);
		 //trigger_free = adc_d;
              }
           }
           trigger_free_Vec.push_back(trigger_free);
	   //---------------------------
	   for (auto auxpad : *auxPadArray) {
              if (auxpad.GetAuxName().compare(std::string("mesh")) == 0) {
                 std::cout << " Auxiliary pad name " << auxpad.GetAuxName() << "\n";
                 Double_t *adc_e = auxpad.GetADC();
		 mesh = GetMaximum(adc_e);
		 //mesh = adc_e;
              }
           }
           mesh_Vec.push_back(mesh);
	   //---------------------------
	   for (auto auxpad : *auxPadArray) {
              if (auxpad.GetAuxName().compare(std::string("mesh_MCA")) == 0) {
                 std::cout << " Auxiliary pad name " << auxpad.GetAuxName() << "\n";
                 Double_t *adc_f = auxpad.GetADC();
		 mesh_MCA = GetMaximum(adc_f);
		 //mesh_MCA = adc_f;
              }
           }
           mesh_MCA_Vec.push_back(mesh_MCA);
	   //---------------------------
           evtNum_vs_trkNum->Fill(i, patternTrackCand.size());

           for (auto track : patternTrackCand) {

              std::cout << " Track " << track.GetTrackID() << " with " << track.GetHitClusterArray()->size()     << " clusters "<< "\n";

              trackID = track.GetTrackID();
              trackIDVec.push_back(trackID);

              if (track.GetIsNoise() || track.GetHitClusterArray()->size() < 5) {
                 std::cout << cRED << " Track is noise or has less than 5 clusters! " << cNORMAL << "\n";
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

	      std::cout<<" Theta : "<<theta*TMath::RadToDeg()<<" Phi : "<<phi*TMath::RadToDeg()<<"\n";
	      
              auto hitClusterArray = track.GetHitClusterArray();
              AtHitCluster iniCluster;
              Double_t zIniCal = std::sqrt(-1);
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
                 iniCluster = hitClusterArray->back();
		 // NB: Use back because We do not reverse the cluster vector like in AtGenfit!
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
              // Double_t pVal = std::sqrt(-1);
              Double_t bChi2 = 0, fChi2 = 0, bNdf = 0, fNdf = 0;
              Double_t distance = std::sqrt(-1);;
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
                       Double_t ex_energy_exp = kine_2b(M_10B, M_D, m_b, m_B, Ebeam_buff, thetaA, E * 1000);
                       EFitXtr = 1000.0 * (TMath::Sqrt(TMath::Power(mom_ext.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener);
                       ExXtr = kine_2b(M_10B, M_D, m_b, m_B, Ebeam_buff, thetaA, EFitXtr);

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

           } // track loop

           outputTree->Fill();

        } // if pattern event

      } // Event

   } // File

   outputFile->cd();
   // outputTree->Print();
   outputTree->Write();
   outputFile->Close();
   return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////
std::tuple<Double_t, Double_t> GetMomFromBrho(Double_t M, Double_t Z, Double_t brho){

   const Double_t M_Ener = M * Conv_AMU_to_MeV / 1000.0;
   Double_t p = brho * Z * (2.99792458 / 10.0); // In GeV
   Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
   std::cout << " Brho : " << brho << " - p : " << p << " - E : " << E << "\n";
   return std::make_tuple(p, E);
}
///////////////////////////////////////////////////////////////////////////////////////////
double kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject){

   // in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
   double Et1 = K_proj + m1;
   double Et2 = m2;
   double Et3 = K_eject + m3;
   double Et4 = Et1 + Et2 - Et3;
   double m4_ex, Ex, theta_cm;
   double s, t, u; //---Mandelstam variables

   s = pow(m1, 2) + pow(m2, 2) + 2 * m2 * Et1;
   u = pow(m2, 2) + pow(m3, 2) - 2 * m2 * Et3;

   m4_ex = sqrt((cos(thetalab) * omega(s, pow(m1, 2),pow(m2, 2))*omega(u, pow(m2, 2),pow(m3, 2)) -
                 (s - pow(m1,2)-pow(m2, 2))*(pow(m2, 2) + pow(m3, 2) - u)) /(2 * pow(m2, 2)) + s + u - pow(m2, 2));
   Ex = m4_ex - m4;

   t = pow(m2, 2) + pow(m4_ex, 2) - 2 * m2 * Et4;

   // for inverse kinematics Note: this angle corresponds to the recoil
   theta_cm = TMath::Pi() - acos((pow(s, 2) + s * (2 * t - pow(m1, 2) - pow(m2, 2) - pow(m3, 2) - pow(m4_ex, 2)) +
                                  (pow(m1, 2) - pow(m2, 2)) * (pow(m3, 2) - pow(m4_ex, 2))) /
                                 (omega(s, pow(m1, 2), pow(m2, 2)) * omega(s, pow(m3, 2), pow(m4_ex, 2))));

   // THcm = theta_cm*TMath::RadToDeg();
   return Ex;
}
///////////////////////////////////////////////////////////////////////////////////////////
double GetMaximum(double *adc){

   double max = 0;

   for (int indTB = 50; indTB < 90; ++indTB) {//change it to 50-70//0-512
      // std::cout<<" TB "<<indTB<<" adc "<<adc[indTB]<<"\n";
      if (adc[indTB] > max)
         max = adc[indTB];
   }

   return max;
}

//---------------------------------
double GetNPeaks(double *adc_test){
  //Int_t npeaks =2;
  //TSpectrum *s = 0;
  //if(s!=0){delete s;}

  TH1D *h1_test = new TH1D("h1_test","h1_test",512,0,511);
  
  for (int iTB = 0; iTB < 512; ++iTB) {
    h1_test->Fill(adc_test[iTB]);
  }
  
  //TSpectrum *s = new TSpectrum(npeaks+1);
  TSpectrum *s = new TSpectrum();
  Int_t nfound = s->Search(h1_test,2," ",0.25);//2 and 0.15
 
  delete s;
  delete h1_test;
  return nfound;
}
//---------------------------------
double GetNPeaksHRS(double *adc_test){

  TSpectrum *s = new TSpectrum();
  Double_t dest[512];
  
  //Int_t nfound = s->Search(h1_test,2," ",0.25);//2 and 0.15
  Int_t nfound;
  //nfound = s->SearchHighRes(adc_test, dest, 512, 8, 2, kFALSE, 1, kFALSE, 1);
  nfound = s->SearchHighRes(adc_test, dest, 512, 8, 2, kTRUE, 3, kTRUE, 3);
  return nfound; 
}

///////////////////////////
