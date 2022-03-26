#include "e20009Ana.h"

#include "AtFitter.h"
#include "AtGenfit.h"

#include <memory>
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

   bool enableMerging = 0;

   // File for checking fit quality
   std::ios_base::openmode mode;
   // mode = std::ios_base::app;
   mode = std::ios_base::out;
   std::ofstream outputFileFit("checkFit.txt", mode);

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
      std::cout << " Simulation ? " << simulationConv << "\n ";
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

   if (simulationConv)
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

   AtFITTER::AtFitter *fFitter =
      new AtFITTER::AtGenfit(magneticField, 0.00001, 1000.0, eLossFileNameWithPath.Data(), gasMediumDensity);
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
   Int_t particleQ;

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
   std::vector<Int_t> particleQVec;
   std::vector<Float_t> POCAOrbZVec;
   std::vector<Float_t> firstOrbZVec;
   std::vector<Float_t> phiOrbZVec;
   std::vector<Float_t> lengthOrbZVec;
   std::vector<Float_t> eLossOrbZVec;

   TString simFile;

   if (simulationConv)
      simFile = "_sim_";
   else
      simFile = "";

   TString outputFileName = "fit_analysis_" + simFile + inputFileName;
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
   outputTree->Branch("fChi2Vec", &fChi2Vec);
   outputTree->Branch("bChi2Vec", &bChi2Vec);
   outputTree->Branch("fNdfVec", &fNdfVec);
   outputTree->Branch("bNdfVec", &bNdfVec);
   outputTree->Branch("ICEVec", &ICEVec);
   outputTree->Branch("particleQVec", &particleQVec);
   outputTree->Branch("POCAOrbZVec", &POCAOrbZVec);
   outputTree->Branch("firstOrbZVec", &firstOrbZVec);
   outputTree->Branch("phiOrbZVec", &phiOrbZVec);
   outputTree->Branch("lengthOrbZVec", &lengthOrbZVec);
   outputTree->Branch("eLossOrbZVec", &eLossOrbZVec);

   for (auto iFile = 0; iFile < files.size(); ++iFile) {

      // fileNameWithPath = dir + filePath + files.at(iFile).Data();
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
         particleQ = -10;

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
         particleQVec.clear();
         POCAOrbZVec.clear();
         firstOrbZVec.clear();
         phiOrbZVec.clear();
         lengthOrbZVec.clear();
         eLossOrbZVec.clear();

         std::cout << cGREEN << " Event Number : " << i << cNORMAL << "\n";

         Reader1.Next();

         AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);
         AtEvent *event = (AtEvent *)evArray->At(0);

         if (patternEvent) {

            auto auxPadArray = event->GetAuxPadArray();
            std::cout << " Number of auxiliary pads : " << auxPadArray.size() << "\n";

            std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
            std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";

            std::vector<AtTrack> mergedTrackPool;
            std::vector<AtTrack> junkTrackPool;
            std::vector<AtTrack> candTrackPool;

            for (auto auxpad : auxPadArray) {
               if (auxpad.GetAuxName().compare(std::string("IC_sca")) == 0) {
                  std::cout << " Auxiliary pad name " << auxpad.GetAuxName() << "\n";
                  Double_t *adc = const_cast<double*>(auxpad.GetADC().data());
		  
                  ICMult = GetNPeaksHRS(&ICTimeVec, &ICVec, adc);
               }
               if (auxpad.GetAuxName().compare(std::string("IC")) == 0) {
                  const Double_t *adc = auxpad.GetADC().data();

                  for (auto iadc = 0; iadc < 512; ++iadc)
                     ICEVec.push_back(adc[iadc]);
               }
            }

            // ICVec.push_back(IC);

            evtNum_vs_trkNum->Fill(i, patternTrackCand.size());

            for (auto track : patternTrackCand) {

               std::cout << " Track " << track.GetTrackID() << " with " << track.GetHitClusterArray()->size()
                         << " clusters "
                         << "\n";

               trackID = track.GetTrackID();
               trackIDVec.push_back(trackID);

               if (track.GetIsNoise() || track.GetHitClusterArray()->size() < 3) {
                  std::cout << cRED << " Track is noise or has less than 3 clusters! " << cNORMAL << "\n";
                  continue;
               }

               // Track merging
               Double_t theta = track.GetGeoTheta();
               std::pair<Double_t, Double_t> center = track.GetGeoCenter();
               Double_t thetaConv;
               if (simulationConv) {
                  thetaConv = 180.0 - theta * TMath::RadToDeg();
               } else {
                  thetaConv = theta * TMath::RadToDeg();
               }

               auto hitClusterArray = track.GetHitClusterArray();
               AtHitCluster iniCluster;
               AtHitCluster secCluster;
               AtHitCluster endCluster;
               Double_t zIniCal = 0;
               Double_t zEndCal = 0;
	       ROOT::Math::XYZPoint iniPos;
               ROOT::Math::XYZPoint secPos;
               ROOT::Math::XYZPoint endPos;

               if (thetaConv < 90.0) {
                  iniCluster =
                     hitClusterArray
                        ->back(); // NB: Use back because We do not reverse the cluster vector like in AtGenfit!
                  secCluster = hitClusterArray->at(hitClusterArray->size() - 2);
                  iniPos = iniCluster.GetPosition();
                  secPos = secCluster.GetPosition();
                  endCluster = hitClusterArray->front();
                  endPos = endCluster.GetPosition();
                  zIniCal = 1000.0 - iniPos.Z();
                  zEndCal = 1000.0 - endPos.Z();
               } else if (thetaConv > 90.0) {
                  iniCluster = hitClusterArray->front();
                  secCluster = hitClusterArray->at(1);
                  iniPos = iniCluster.GetPosition();
                  secPos = secCluster.GetPosition();
                  endCluster = hitClusterArray->back();
                  endPos = endCluster.GetPosition();
                  zIniCal = iniPos.Z();
                  zEndCal = endPos.Z();
               }

               xiniPRA = iniPos.X();
               yiniPRA = iniPos.Y();
               ziniPRA = zIniCal;

               // NB: Mind the x sign!
               std::cout << " Old phi from PRA : " << track.GetGeoPhi() * TMath::RadToDeg() << "\n";
               Double_t phiClus = TMath::ATan2(secPos.Y() - iniPos.Y(), -secPos.X() + iniPos.X());
               track.SetGeoPhi(-phiClus);

               // This is just to select distances
               std::cout << " Initial position : " << xiniPRA << " - " << yiniPRA << " - " << ziniPRA << "\n";
               std::cout << " Second position : " << secPos.X() << " - " << secPos.Y() << " - " << secPos.Z() << "\n";
               std::cout << " End position : " << endPos.X() << " - " << endPos.Y() << " - " << zEndCal << "\n";
               std::cout << " Theta (convention) : " << thetaConv << " - Phi Clus : " << phiClus * TMath::RadToDeg()
                         << "\n";
               std::cout << " Track center - X :  " << center.first << " - Y : " << center.second << "\n";
               std::cout << " Track phi recalc : " << track.GetGeoPhi() * TMath::RadToDeg() << "\n";

               // Skip tracks that are far from Z (to be checked against number of iterations for extrapolation)
               Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());

               std::cout << cRED << " Distance to Z " << dist << cNORMAL << "\n";
               if (dist > 100.0) { // mm
                  junkTrackPool.push_back(track);
               } else
                  candTrackPool.push_back(track);
            }

            if (enableMerging)
               fFitter->MergeTracks(&candTrackPool, &junkTrackPool, &mergedTrackPool, fitDirection, simulationConv);
            else {
               mergedTrackPool = candTrackPool;
            }

            for (auto track : mergedTrackPool) {

               std::cout << " Merge Tracks Pool Size : " << mergedTrackPool.size() << "\n";

               if (enableMerging) {
                  std::cerr << cRED << " Error! Phi calculation after merging not implemented yet! Exiting..."
                            << "\n";
                  std::exit(0);
                  track.ResetHitClusterArray();
                  ClusterizeSmooth3D(track, 10.0, 30.0); // Reclusterizing
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
	       ROOT::Math::XYZPoint iniPos;

               // for (auto cluster : *hitClusterArray) {
               // TVector3 pos = cluster.GetPosition();
               // std::cout<<pos.X()<<"     "<<pos.Y()<<"   "<<pos.Z()<<"\n";
               // }

               // Variable for convention (simulation comes reversed)
               Double_t thetaConv;
               if (simulationConv) {
                  thetaConv = 180.0 - theta * TMath::RadToDeg();
               } else {
                  thetaConv = theta * TMath::RadToDeg();
               }

               if (thetaConv < 90.0) {
                  iniCluster =
                     hitClusterArray
                        ->back(); // NB: Use back because We do not reverse the cluster vector like in AtGenfit!
                  iniPos = iniCluster.GetPosition();
                  zIniCal = 1000.0 - iniPos.Z();
               } else if (thetaConv > 90.0) {
                  iniCluster = hitClusterArray->front();
                  iniPos = iniCluster.GetPosition();
                  zIniCal = iniPos.Z();
               }

               xiniPRA = iniPos.X();
               yiniPRA = iniPos.Y();
               ziniPRA = zIniCal;

               // This is just to select distances
               std::cout << " Initial position : " << xiniPRA << " - " << yiniPRA << " - " << ziniPRA << "\n";

               // Fit
               if (fitDirection == 0 && thetaConv > 90) // O is between 0 and 90 (simulation goes from 90 to 180) (d,d)
                  continue;
               else if (fitDirection == 1 &&
                        thetaConv < 90) // 1 is between 90 and 180 (simulation goes from 0 to 90) (d,p)
                  continue;

               // Skip border angles
               if (theta * TMath::RadToDeg() < 5 || theta * TMath::RadToDeg() > 170)
                  continue;

               // Skip tracks that are far from Z (to be checked against number of iterations for extrapolation)
               Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());

               std::cout << cRED << " Distance to Z (Candidate Track Pool) " << dist << cNORMAL << "\n";

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
               Int_t nSteps = 0;

               // First orbit
               Double_t POCAOrbZ = 1E6;
               Double_t firstOrbZ = 0.0;
               Double_t phiOrbZ = 0.0;
               Double_t lengthOrbZ = 0.0;
               Double_t eLossOrbZ = 0.0;

               try {

                  if (fitTrack && fitTrack->hasKalmanFitStatus()) {

                     auto KalmanFitStatus = fitTrack->getKalmanFitStatus();
                     auto trackRep = fitTrack->getTrackRep(0); // Only one representation is sved for the moment.

                     if (KalmanFitStatus->isFitConverged(false)) {
                        // KalmanFitStatus->Print();
                        genfit::MeasuredStateOnPlane fitState = fitTrack->getFittedState();
                        particleQ = fitState.getCharge();

                        fChi2 = KalmanFitStatus->getForwardChi2();
                        bChi2 = KalmanFitStatus->getBackwardChi2();
                        fNdf = KalmanFitStatus->getForwardNdf();
                        bNdf = KalmanFitStatus->getBackwardNdf();
                        // fitState.Print();
                        fitState.getPosMomCov(pos_res, mom_res, cov_res);
                        trackLength = KalmanFitStatus->getTrackLen();
                        pVal = KalmanFitStatus->getPVal();

                        // fKalmanFitter -> getChiSquNdf(gfTrack, trackRep, bChi2, fChi2, bNdf, fNdf);
                        Float_t stepXtr = -0.01;
                        Int_t minCnt = 0;
                        Int_t minCntExt = 0;

                        TVector3 pos_ini_buff = pos_res;
                        Double_t length = 0.0;

                        try {
                           for (auto iStep = 0; iStep < 200; ++iStep) {

                              trackRep->extrapolateBy(fitState, stepXtr * iStep);
                              mom_ext_buff = fitState.getMom();
                              pos_ext_buff = fitState.getPos();

                              length += (pos_ext_buff - pos_ini_buff).Mag();
                              pos_ini_buff = pos_ext_buff;

                              double distance =
                                 TMath::Sqrt(pos_ext_buff.X() * pos_ext_buff.X() + pos_ext_buff.Y() * pos_ext_buff.Y());
                              // if (fVerbosityLevel > 2){
                              /*std::cout << cYELLOW << " Extrapolation: Total Momentum : " << mom_ext_buff.Mag()
                                        << " - Position : " << pos_ext_buff.X() << "  " << pos_ext_buff.Y() << "  "
                                        << pos_ext_buff.Z() << " - distance of approach : " << distance <<" - length :
                                 "<<length<< cNORMAL << "\n";*/
                              //}

                              // if(pos_ext_buff.Z()<0)
                              // break;

                              if (distance < POCA) {
                                 POCA = distance;
                                 POCAXtr = distance;
                                 mom_ext = mom_ext_buff;
                                 pos_ext = pos_ext_buff;
                                 distXtr = iStep * stepXtr;
                                 ++minCnt;
                                 minCntExt = 0;
                                 nSteps = iStep;
                              }
                              // Loop control
                              // if(minCntExt>20) //Break the loop if a new minimum is not found after several
                              // iterations break;

                              ++minCntExt;
                           } // Extrapolation loop

                           // Find the first orbit
                           firstOrbit fOrbit = GetFirstOrbit(fitTrack, trackRep, pos_ext);
                           POCAOrbZ = fOrbit.POCA;
                           firstOrbZ = fOrbit.Z;
                           phiOrbZ = fOrbit.phi;
                           lengthOrbZ = fOrbit.length;
                           eLossOrbZ = fOrbit.eLoss;

                        } catch (genfit::Exception &e) {
                           mom_ext.SetXYZ(0, 0, 0);
                           pos_ext.SetXYZ(0, 0, 0);
                        } // try

                        // mom_res = mom_ext;
                        // pos_res = pos_ext;
                        xiniFitXtr = pos_ext.X();
                        yiniFitXtr = pos_ext.Y();
                        ziniFitXtr = pos_ext.Z();

                        std::cout << cYELLOW << " Extrapolation: Total Momentum : " << mom_ext.Mag()
                                  << " - Position : " << pos_ext.X() << "  " << pos_ext.Y() << "  " << pos_ext.Z()
                                  << " - POCA : " << POCA << " - Steps : " << nSteps << cNORMAL << "\n";

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

                        outputFileFit << i << " " << E << " " << EFitXtr << " " << AFit << " " << PhiFit << " "
                                      << particleQ << " " << fChi2 << " " << fNdf << "\n";

                     } // Kalman fit
                  }    // Kalman status
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

               particleQVec.push_back(particleQ);

               POCAOrbZVec.push_back(POCAOrbZ);
               firstOrbZVec.push_back(firstOrbZ);
               phiOrbZVec.push_back(phiOrbZ);
               lengthOrbZVec.push_back(lengthOrbZ);
               eLossOrbZVec.push_back(eLossOrbZ);

            } // track loop

            outputTree->Fill();

         } // if pattern event

      } // Event

   } // File

   outputFileFit.close();

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
      Kine_AngRec_EnerRec_9AMeV->SetLineColor(kRed + 1);
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

firstOrbit GetFirstOrbit(genfit::Track *track, genfit::AbsTrackRep *rep, TVector3 vertex)
{
   genfit::KalmanFitterInfo *fi;
   genfit::KalmanFitterInfo *prevFi = 0;
   unsigned int numhits = track->getNumPointsWithMeasurement();
   const genfit::MeasuredStateOnPlane *fittedState(nullptr);
   const genfit::MeasuredStateOnPlane *prevFittedState(nullptr);

   std::vector<TVector3> xtrTrack;
   std::vector<trackSegment> segments;

   // Loop over hits
   for (unsigned int j = 0; j < numhits; j++) { // loop over all hits in the track

      // std::cout<<" Hit number "<<j<<"\n";

      fittedState = nullptr;

      genfit::TrackPoint *tp = track->getPointWithMeasurement(j);
      if (!tp->hasRawMeasurements()) {
         std::cerr << "trackPoint has no raw measurements" << std::endl;
         continue;
      }

      genfit::AbsFitterInfo *fitterInfo = tp->getFitterInfo(rep);
      fi = dynamic_cast<genfit::KalmanFitterInfo *>(fitterInfo);

      // get the fitter infos ------------------------------------------------------------------
      if (!tp->hasFitterInfo(rep)) {
         std::cerr << "trackPoint has no fitterInfo for rep" << std::endl;
         continue;
      }

      if (!fi->hasPredictionsAndUpdates()) {
         std::cerr << "KalmanFitterInfo does not have all predictions and updates" << std::endl;
         // continue;
      } else {
         try {
            fittedState = &(fi->getFittedState(true));
         } catch (genfit::Exception &e) {
            std::cerr << e.what();
            std::cerr << "can not get fitted state" << std::endl;
            fittedState = nullptr;
            prevFi = fi;
            prevFittedState = fittedState;
            continue;
         }
      }

      if (fittedState == nullptr) {
         if (fi->hasForwardUpdate()) {
            fittedState = fi->getForwardUpdate();
         } else if (fi->hasBackwardUpdate()) {
            fittedState = fi->getBackwardUpdate();
         } else if (fi->hasForwardPrediction()) {
            fittedState = fi->getForwardPrediction();
         } else if (fi->hasBackwardPrediction()) {
            fittedState = fi->getBackwardPrediction();
         }
      }

      if (fittedState == nullptr) {
         std::cout << "cannot get any state from fitterInfo, continue.\n";
         prevFi = fi;
         prevFittedState = fittedState;
         continue;
      }

      TVector3 pos = fittedState->getPos();
      TVector3 mom = fittedState->getMom();
      double charge = fittedState->getCharge();

      ConstructTrack(prevFittedState, fittedState, rep, xtrTrack, segments);

      /*
      unsigned int nMeas = fi->getNumMeasurements();
      for (unsigned int iMeas = 0; iMeas < nMeas; ++iMeas) {

        const genfit::MeasurementOnPlane* mop = fi->getMeasurementOnPlane(iMeas);
   const genfit::SharedPlanePtr& sharedPlane = mop->getPlane();
   //mop->Print();
   TVector3 pos(0,0,0), mom(0,0,0);
   TMatrixDSym cov;
   const TVector3& _o = sharedPlane->getO();
   const TVector3& _u = sharedPlane->getU();
        const TVector3& _v = sharedPlane->getV();
   //sharedPlane->Print();
   //mop->getPosMomCov(pos,mom,cov);
        //mop->getRep()->getPosMom(mop, pos, mom);
      //std::cout<<" iMeas "<<iMeas<<"\n";




   } // Measurement loop*/

      prevFi = fi;
      prevFittedState = fittedState;

   } // Loop over hits

   // Determination of first orbit
   Double_t deltaPhi = 0.0;
   Double_t firstOrbZ = 0.0;
   Double_t POCAOrbZ = 1E6;
   Double_t phiOrbZ = 0.0;
   Double_t lengthOrbZ = 0;
   Double_t length = 0;
   Double_t eLossOrbZ = 0.0;
   Double_t eLoss = 0;

   if (segments.size() > 0)
      eLoss += segments.at(0).eLoss;

   for (auto iseg = 1; iseg < segments.size(); ++iseg) {

      auto prevSegment = segments.at(iseg - 1);
      auto segment = segments.at(iseg);
      // std::cout<< prevSegment;
      // std::cout<< segment;
      auto prevSegPos = prevSegment.iniPos;
      auto dir = segment.iniPos - prevSegment.iniPos;
      length += dir.Mag();
      eLoss += prevSegment.eLoss;
      Double_t phiInc = prevSegment.phi - segment.phi;
      Double_t distance = TMath::Sqrt(prevSegPos.X() * prevSegPos.X() + prevSegPos.Y() * prevSegPos.Y());
      phiInc = (phiInc > 0) ? phiInc : 2.0 * TMath::Pi() + phiInc;
      deltaPhi += phiInc;

      // std::cout<<phiInc*TMath::RadToDeg()<<"    "<<deltaPhi*TMath::RadToDeg()<<"\n";
      // std::cout<<" Distance to z axis : "<<distance<<"\n";
      // std::cout<<" Length : "<<length<<"\n";
      // std::cout<<" Z projection : "<<prevSegPos.Z()<<"\n";
      // std::cout<<" momLoss "<<momLoss<<"\n";

      // NB: Find poca after at least half turn.
      if (distance < POCAOrbZ && deltaPhi > TMath::Pi()) {
         POCAOrbZ = distance;
         firstOrbZ = prevSegPos.Z();
         phiOrbZ = deltaPhi;
         lengthOrbZ = length;
         eLossOrbZ = eLoss;
      }
   }

   firstOrbit fOrbit = {POCAOrbZ, firstOrbZ, phiOrbZ, lengthOrbZ, eLossOrbZ};

   // std::cout<<" POCAOrbZ : "<<POCAOrbZ<<" - firstOrbZ : "<<firstOrbZ<<" - phiOrbZ : "<<phiOrbZ*TMath::RadToDeg()<<"
   // lengthOrbZ : "<<lengthOrbZ<<" - eLossOrbZ : "<<eLossOrbZ<<"\n";

   return fOrbit;
}

void ConstructTrack(const genfit::StateOnPlane *prevState, const genfit::StateOnPlane *state,
                    const genfit::AbsTrackRep *rep, std::vector<TVector3> &track, std::vector<trackSegment> &segments)
{

   if (prevState == nullptr || state == nullptr) {
      // std::cerr << "prevState == nullptr || state == nullptr\n";
      return;
   }

   TVector3 pos, dir, oldPos, oldDir;
   TVector3 mom, mdir, oldMom, oldmDir;
   rep->getPosDir(*state, pos, dir);
   rep->getPosDir(*prevState, oldPos, oldDir);
   mom = state->getMom();
   oldMom = prevState->getMom();
   Double_t momLoss = (mom - oldMom).Mag();

   double distA = (pos - oldPos).Mag();
   double distB = distA;
   if ((pos - oldPos) * oldDir < 0)
      distA *= -1.;
   if ((pos - oldPos) * dir < 0)
      distB *= -1.;
   TVector3 intermediate1 = oldPos + 0.3 * distA * oldDir;
   TVector3 intermediate2 = pos - 0.3 * distB * dir;

   TVector3 dirToMom = (pos - oldPos);

   // Manually calculating phi
   Double_t phi = TMath::ATan2(pos.Y() - oldPos.Y(), pos.X() - oldPos.X());

   /* std::cout<<" Old Pos "<<oldPos(0)<<" "<<oldPos(1)<<" "<< oldPos(2)<<"\n";
      std::cout<<" Intermediate 1 "<<intermediate1(0)<<" "<<intermediate1(1)<<" "<< intermediate1(2)<<"\n";
      std::cout<<" Intermediate 2 "<<intermediate2(0)<<" "<<intermediate2(1)<<" "<< intermediate2(2)<<"\n";
      std::cout<<" Pos "<<pos(0)<<" "<<pos(1)<<" "<<pos(2)<<"\n";
      std::cout<<" Direction angles : "<<"\n";
      std::cout<<cYELLOW<<" Dir : "<<dir.Theta()*TMath::RadToDeg()<<" "<<dir.Phi()*TMath::RadToDeg()<<cNORMAL<<"\n";
      std::cout<<cYELLOW<<" DirToMom : "<<dirToMom.Theta()*TMath::RadToDeg()<<"
      "<<dirToMom.Phi()*TMath::RadToDeg()<<cNORMAL<<"\n"; std::cout<<cYELLOW<<" Phi (ATan2) :
      "<<phi*TMath::RadToDeg()<<cNORMAL<<"\n";*/
   // std::cout<<" Old dir : "<<oldDir.Theta()*TMath::RadToDeg()<<" "<<oldDir.Phi()*TMath::RadToDeg()<<"\n";
   // std::cout<<" Momentum Loss "<<1000.0*momLoss<<" MeV "<<"\n";

   // NB Just for testing (d,p)
   Double_t mass = 1.007276466812 * 931.49401 / 1000.0;
   Double_t ELoss2 = 1000.0 * (TMath::Sqrt(TMath::Power((mom).Mag(), 2) + TMath::Power(mass, 2)) - mass);
   Double_t ELoss1 = 1000.0 * (TMath::Sqrt(TMath::Power((oldMom).Mag(), 2) + TMath::Power(mass, 2)) - mass);
   Double_t ELoss = ELoss1 - ELoss2;

   trackSegment segment = {ELoss, pos, dirToMom, dir, dirToMom.Theta(), dirToMom.Phi(), (UInt_t)segments.size()};
   segments.push_back(segment);

   track.push_back(oldPos);
   track.push_back(intermediate1);
   track.push_back(intermediate2);
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

void Clusterize3D(AtTrack &track, Float_t distance, Float_t radius)
{
   auto hitArray = track.GetHitArray();
   std::vector<AtHit> hitTBArray;
   int clusterID = 0;

   // std::cout<<" ================================================================= "<<"\n";
   // std::cout<<" Clusterizing track : "<<track.GetTrackID()<<"\n";

   for (auto iHits = 0; iHits < hitArray.size(); ++iHits) {
      auto pos = hitArray.at(iHits).GetPosition();
      double Q = hitArray.at(iHits).GetCharge();
      int TB = hitArray.at(iHits).GetTimeStamp();
      std::cout << " Pos : " << pos.X() << " - " << pos.Y() << " - " << pos.Z() << " - TB : " << TB
                << " - Charge : " << Q << "\n";
   }
   // Diffusion coefficients (TODO: Get them from the parameter file)
   Double_t driftVel = 1.0;       // cm/us
   Double_t samplingRate = 0.320; // us
   Double_t d_t = 0.0009;         // cm^2/us
   Double_t d_l = 0.0009;         // cm^2/us
   Double_t D_T = TMath::Sqrt((2.0 * d_t) / driftVel);
   Double_t D_L = TMath::Sqrt((2.0 * d_l) / driftVel);

   if (hitArray.size() > 0) {

      auto refPos = hitArray.at(0).GetPosition(); // First hit
      // TODO: Create a clustered hit from the very first hit (test)

      for (auto iHit = 0; iHit < hitArray.size(); ++iHit) {

         auto hit = hitArray.at(iHit);

         // Check distance with respect to reference Hit
         Double_t distRef = TMath::Sqrt((hit.GetPosition() - refPos).Mag2());

         if (distRef < distance) {

            continue;

         } else {

            // std::cout<<" Clustering "<<iHit<<" of "<<hitArray.size()<<"\n";
            // std::cout<<" Distance to reference : "<<distRef<<"\n";
            // std::cout<<" Reference position : "<<refPos.X()<<" - "<<refPos.Y()<<" - "<<refPos.Z()<<" -
            // "<<refPos.Mag()<<"\n";

            Double_t clusterQ = 0.0;
            hitTBArray.clear();
            std::copy_if(
               hitArray.begin(), hitArray.end(), std::back_inserter(hitTBArray),
               [&refPos, radius](AtHit &hitIn) { return TMath::Sqrt((hitIn.GetPosition() - refPos).Mag2()) < radius; });

            // std::cout<<" Clustered "<<hitTBArray.size()<<" Hits "<<"\n";

            if (hitTBArray.size() > 0) {
               double x = 0, y = 0, z = 0;
               double sigma_x = 0, sigma_y = 0, sigma_z = 0;

               int timeStamp;
               std::shared_ptr<AtHitCluster> hitCluster = std::make_shared<AtHitCluster>();
               hitCluster->SetClusterID(clusterID);
               Double_t hitQ = 0.0;
               std::for_each(hitTBArray.begin(), hitTBArray.end(),
                             [&x, &y, &z, &hitQ, &timeStamp, &sigma_x, &sigma_y, &sigma_z, &D_T, &D_L, &driftVel,
                              &samplingRate](AtHit &hitInQ) {
                                auto pos = hitInQ.GetPosition();
                                x += pos.X() * hitInQ.GetCharge();
                                y += pos.Y() * hitInQ.GetCharge();
                                z += pos.Z();
                                hitQ += hitInQ.GetCharge();
                                timeStamp += hitInQ.GetTimeStamp();

                                // Calculation of variance (DOI: 10.1051/,00010 (2017)715001EPJ Web of
                                // Conferences50epjconf/2010010)
                                sigma_x += hitInQ.GetCharge() *
                                           TMath::Sqrt(TMath::Power(0.2, 2) +
                                                       pos.Z() * TMath::Power(D_T, 2)); // 0.2 mm of position resolution
                                sigma_y += sigma_x;
                                sigma_z += TMath::Sqrt((1.0 / 6.0) * TMath::Power(driftVel * samplingRate, 2) +
                                                       pos.Z() * TMath::Power(D_L, 2));
                             });
               x /= hitQ;
               y /= hitQ;
               z /= hitTBArray.size();
               timeStamp /= std::round(timeStamp);

               sigma_x /= hitQ;
               sigma_y /= hitQ;
               sigma_z /= hitTBArray.size();

	       ROOT::Math::XYZPoint clustPos(x, y, z);
               Bool_t checkDistance = kTRUE;

               // Check distance with respect to existing clusters
               for (auto iClusterHit : *track.GetHitClusterArray()) {
                  if (TMath::Sqrt((iClusterHit.GetPosition() - clustPos).Mag2()) < distance) {
                     // std::cout<<" Cluster with less than  : "<<distance<<" found "<<"\n";
                     checkDistance = kFALSE;
                     continue;
                  }
               }

               if (checkDistance) {
                  hitCluster->SetCharge(hitQ);
                  hitCluster->SetPosition(x, y, z);
                  hitCluster->SetTimeStamp(timeStamp);
                  TMatrixDSym cov(3); // TODO: Setting covariant matrix based on pad size and drift time resolution.
                                      // Using estimations for the moment.
                  cov(0, 1) = 0;
                  cov(1, 2) = 0;
                  cov(2, 0) = 0;
                  cov(0, 0) = TMath::Power(sigma_x, 2); // 0.04;
                  cov(1, 1) = TMath::Power(sigma_y, 2); // 0.04;
                  cov(2, 2) = TMath::Power(sigma_z, 2); // 0.01;
                  hitCluster->SetCovMatrix(cov);
                  ++clusterID;
                  track.AddClusterHit(hitCluster);
               }
            }
         }

         // Sanity check
         /*std::cout<<" Hits for cluster "<<iHit<<" centered in "<<refPos.X()<<" - "<<refPos.Y()<<"-"<<refPos.Z()<<"\n";
    for(auto iHits=0;iHits<hitTBArray.size();++iHits)
         {
           TVector3 pos    = hitTBArray.at(iHits).GetPosition();
           double Q = hitTBArray.at(iHits).GetCharge();
           int TB          = hitTBArray.at(iHits).GetTimeStamp();
           std::cout<<" Pos : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<" - TB : "<<TB<<" - Charge : "<<Q<<"\n";
      std::cout<<" Distance to cluster center "<<TMath::Abs((track.GetHitClusterArray()->back().GetPosition() -
    pos).Mag())<<"\n";
    }
         std::cout<<"=================================================="<<"\n";*/

         refPos = hitArray.at(iHit).GetPosition();

         //} // if distance

      } // for

   } // if array size
}

void ClusterizeSmooth3D(AtTrack &track, Float_t distance, Float_t radius)
{
   auto hitArray = track.GetHitArray();
   std::vector<AtHit> hitTBArray;
   int clusterID = 0;

   // std::cout<<" ================================================================= "<<"\n";
   // std::cout<<" Clusterizing track : "<<track.GetTrackID()<<"\n";

   /*for(auto iHits=0;iHits<hitArray.size();++iHits)
     {
       TVector3 pos    = hitArray.at(iHits).GetPosition();
       double Q = hitArray.at(iHits).GetCharge();
       int TB          = hitArray.at(iHits).GetTimeStamp();
       //std::cout<<" Pos : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<" - TB : "<<TB<<" - Charge : "<<Q<<"\n";
       }*/

   // Diffusion coefficients (TODO: Get them from the parameter file)
   Double_t driftVel = 1.0;       // cm/us
   Double_t samplingRate = 0.320; // us
   Double_t d_t = 0.0009;         // cm^2/us
   Double_t d_l = 0.0009;         // cm^2/us
   Double_t D_T = TMath::Sqrt((2.0 * d_t) / driftVel);
   Double_t D_L = TMath::Sqrt((2.0 * d_l) / driftVel);

   if (hitArray.size() > 0) {
      
      auto refPos = hitArray.at(0).GetPosition(); // First hit
      // TODO: Create a clustered hit from the very first hit (test)

      for (auto iHit = 0; iHit < hitArray.size(); ++iHit) {

         auto hit = hitArray.at(iHit);

         // Check distance with respect to reference Hit
         Double_t distRef = TMath::Sqrt((hit.GetPosition() - refPos).Mag2());

         if (distRef < distance && iHit != 0) {

            continue;

         } else {

            // std::cout<<" Clustering "<<iHit<<" of "<<hitArray.size()<<"\n";
            // std::cout<<" Distance to reference : "<<distRef<<"\n";
            // std::cout<<" Reference position : "<<refPos.X()<<" - "<<refPos.Y()<<" - "<<refPos.Z()<<" -
            // "<<refPos.Mag()<<"\n";

            Double_t clusterQ = 0.0;
            hitTBArray.clear();
            std::copy_if(
               hitArray.begin(), hitArray.end(), std::back_inserter(hitTBArray),
               [&refPos, radius](AtHit &hitIn) { return TMath::Sqrt((hitIn.GetPosition() - refPos).Mag2()) < radius; });

            // std::cout<<" Clustered "<<hitTBArray.size()<<" Hits "<<"\n";

            if (hitTBArray.size() > 0) {
               double x = 0, y = 0, z = 0;
               double sigma_x = 0, sigma_y = 0, sigma_z = 0;

               int timeStamp;
               std::shared_ptr<AtHitCluster> hitCluster = std::make_shared<AtHitCluster>();
               hitCluster->SetClusterID(clusterID);
               Double_t hitQ = 0.0;
               std::for_each(hitTBArray.begin(), hitTBArray.end(),
                             [&x, &y, &z, &hitQ, &timeStamp, &sigma_x, &sigma_y, &sigma_z, &D_T, &D_L, &driftVel,
                              &samplingRate](AtHit &hitInQ) {
                                auto pos = hitInQ.GetPosition();
                                x += pos.X() * hitInQ.GetCharge();
                                y += pos.Y() * hitInQ.GetCharge();
                                z += pos.Z();
                                hitQ += hitInQ.GetCharge();
                                timeStamp += hitInQ.GetTimeStamp();

                                // Calculation of variance (DOI: 10.1051/,00010 (2017)715001EPJ Web of
                                // Conferences50epjconf/2010010)
                                sigma_x += hitInQ.GetCharge() *
                                           TMath::Sqrt(TMath::Power(0.2, 2) +
                                                       pos.Z() * TMath::Power(D_T, 2)); // 0.2 mm of position resolution
                                sigma_y += sigma_x;
                                sigma_z += TMath::Sqrt((1.0 / 6.0) * TMath::Power(driftVel * samplingRate, 2) +
                                                       pos.Z() * TMath::Power(D_L, 2));
                             });
               x /= hitQ;
               y /= hitQ;
               z /= hitTBArray.size();
               timeStamp /= std::round(timeStamp);

               sigma_x /= hitQ;
               sigma_y /= hitQ;
               sigma_z /= hitTBArray.size();

	       ROOT::Math::XYZPoint clustPos(x, y, z);
               Bool_t checkDistance = kTRUE;

               // Check distance with respect to existing clusters
               for (auto iClusterHit : *track.GetHitClusterArray()) {
                  if (TMath::Sqrt((iClusterHit.GetPosition() - clustPos).Mag2()) < distance) {
                     // std::cout<<" Cluster with less than  : "<<distance<<" found "<<"\n";
                     checkDistance = kFALSE;
                     continue;
                  }
               }

               if (checkDistance) {
                  hitCluster->SetCharge(hitQ);
                  hitCluster->SetPosition(x, y, z);
                  hitCluster->SetTimeStamp(timeStamp);
                  TMatrixDSym cov(3); // TODO: Setting covariant matrix based on pad size and drift time resolution.
                                      // Using estimations for the moment.
                  cov(0, 1) = 0;
                  cov(1, 2) = 0;
                  cov(2, 0) = 0;
                  cov(0, 0) = TMath::Power(sigma_x, 2); // 0.04;
                  cov(1, 1) = TMath::Power(sigma_y, 2); // 0.04;
                  cov(2, 2) = TMath::Power(sigma_z, 2); // 0.01;
                  hitCluster->SetCovMatrix(cov);
                  ++clusterID;
                  track.AddClusterHit(hitCluster);
               }
            }
         }

         // Sanity check
         /*std::cout<<" Hits for cluster "<<iHit<<" centered in "<<refPos.X()<<" - "<<refPos.Y()<<"-"<<refPos.Z()<<"\n";
    for(auto iHits=0;iHits<hitTBArray.size();++iHits)
         {
           TVector3 pos    = hitTBArray.at(iHits).GetPosition();
           double Q = hitTBArray.at(iHits).GetCharge();
           int TB          = hitTBArray.at(iHits).GetTimeStamp();
           std::cout<<" Pos : "<<pos.X()<<" - "<<pos.Y()<<" - "<<pos.Z()<<" - TB : "<<TB<<" - Charge : "<<Q<<"\n";
      std::cout<<" Distance to cluster center "<<TMath::Abs((track.GetHitClusterArray()->back().GetPosition() -
    pos).Mag())<<"\n";
    }
         std::cout<<"=================================================="<<"\n";*/

         if (iHit == 0)
            continue;

         refPos = hitArray.at(iHit).GetPosition();

         //} // if distance

      } // for

      // Smoothing track
      std::vector<AtHitCluster> *hitClusterArray = track.GetHitClusterArray();
      radius /= 2.0;
      std::vector<std::shared_ptr<AtHitCluster>> hitClusterBuffer;

      // std::cout<<" Hit cluster array size "<<hitClusterArray->size()<<"\n";

      if (hitClusterArray->size() > 2) {

         for (auto iHitCluster = 0; iHitCluster < hitClusterArray->size() - 1;
              ++iHitCluster) // Calculating distances between pairs of clusters
         {

            auto clusBack = hitClusterArray->at(iHitCluster).GetPosition();
            auto clusForw = hitClusterArray->at(iHitCluster + 1).GetPosition();
            auto clusMidPos = (clusBack - clusForw) * 0.5 + clusForw ;
            std::vector<ROOT::Math::XYZPoint> renormClus{clusBack, clusMidPos};

            if (iHitCluster == (hitClusterArray->size() - 2))
               renormClus.push_back(clusForw);

            // Create a new cluster and renormalize the charge of the other with half the radius.
            for (auto iClus : renormClus) {
               hitTBArray.clear();
               std::copy_if(
                  hitArray.begin(), hitArray.end(), std::back_inserter(hitTBArray),
                  [&iClus, radius](AtHit &hitIn) { return TMath::Sqrt((hitIn.GetPosition() - iClus).Mag2()) < radius; });

               if (hitTBArray.size() > 0) {
                  double x = 0, y = 0, z = 0;
                  double sigma_x = 0, sigma_y = 0, sigma_z = 0;

                  int timeStamp;
                  std::shared_ptr<AtHitCluster> hitCluster = std::make_shared<AtHitCluster>();
		  hitCluster->SetClusterID(clusterID);
                  Double_t hitQ = 0.0;
                  std::for_each(hitTBArray.begin(), hitTBArray.end(),
                                [&x, &y, &z, &hitQ, &timeStamp, &sigma_x, &sigma_y, &sigma_z, &D_T, &D_L, &driftVel,
                                 &samplingRate](AtHit &hitInQ) {
                                   auto pos = hitInQ.GetPosition();
                                   x += pos.X() * hitInQ.GetCharge();
                                   y += pos.Y() * hitInQ.GetCharge();
                                   z += pos.Z();
                                   hitQ += hitInQ.GetCharge();
                                   timeStamp += hitInQ.GetTimeStamp();

                                   // Calculation of variance (DOI: 10.1051/,00010 (2017)715001EPJ Web of
                                   // Conferences50epjconf/2010010)
                                   sigma_x +=
                                      hitInQ.GetCharge() *
                                      TMath::Sqrt(TMath::Power(0.2, 2) +
                                                  pos.Z() * TMath::Power(D_T, 2)); // 0.2 mm of position resolution
                                   sigma_y += sigma_x;
                                   sigma_z += TMath::Sqrt((1.0 / 6.0) * TMath::Power(driftVel * samplingRate, 2) +
                                                          pos.Z() * TMath::Power(D_L, 2));
                                });
                  x /= hitQ;
                  y /= hitQ;
                  z /= hitTBArray.size();
                  timeStamp /= std::round(timeStamp);

                  sigma_x /= hitQ;
                  sigma_y /= hitQ;
                  sigma_z /= hitTBArray.size();
		  
                  TVector3 clustPos(x, y, z);
                  hitCluster->SetCharge(hitQ);
                  hitCluster->SetPosition(x, y, z);
                  hitCluster->SetTimeStamp(timeStamp);
                  TMatrixDSym cov(3); // TODO: Setting covariant matrix based on pad size and drift time resolution.
                                      // Using estimations for the moment.
                  cov(0, 1) = 0;
                  cov(1, 2) = 0;
                  cov(2, 0) = 0;
                  cov(0, 0) = TMath::Power(sigma_x, 2); // 0.04;
                  cov(1, 1) = TMath::Power(sigma_y, 2); // 0.04;
                  cov(2, 2) = TMath::Power(sigma_z, 2); // 0.01;
                  hitCluster->SetCovMatrix(cov);
                  ++clusterID;
                  hitClusterBuffer.push_back(hitCluster);

               } // hitTBArray size

            } // for iClus

         } // for HitArray

         // Remove previous clusters
         track.ResetHitClusterArray();

         // Adding new clusters
         for (auto iHitClusterRe : hitClusterBuffer) {

            track.AddClusterHit(iHitClusterRe);
         }

      } // Cluster array size

   } // if array size
}

void Clusterize(AtTrack &track)
{
   // std::cout << " ================================================================= "
   //         << "\n";
   // std::cout << " Clusterizing track : " << track.GetTrackID() << "\n";
   auto hitArray = track.GetHitArray();
   std::vector<AtHit> hitTBArray;
   int clusterID = 0;

   /* for (auto iHits = 0; iHits < hitArray.size(); ++iHits) {
      TVector3 pos = hitArray.at(iHits).GetPosition();
      double Q = hitArray.at(iHits).GetCharge();
      int TB = hitArray.at(iHits).GetTimeStamp();
      std::cout << " Pos : " << pos.X() << " - " << pos.Y() << " - " << pos.Z() << " - TB : " << TB
                << " - Charge : " << Q << "\n";
      }*/

   for (auto iTB = 0; iTB < 512; ++iTB) {

      Double_t clusterQ = 0.0;
      hitTBArray.clear();

      std::copy_if(hitArray.begin(), hitArray.end(), std::back_inserter(hitTBArray),
                   [&iTB](AtHit &hit) { return hit.GetTimeStamp() == iTB; });

      if (hitTBArray.size() > 0) {
         double x = 0, y = 0;
         std::shared_ptr<AtHitCluster> hitCluster = std::make_shared<AtHitCluster>();
	 hitCluster->SetClusterID(clusterID);
         Double_t hitQ = 0.0;
         std::for_each(hitTBArray.begin(), hitTBArray.end(), [&x, &y, &hitQ](AtHit &hit) {
            auto pos = hit.GetPosition();
            x += pos.X() * hit.GetCharge();
            y += pos.Y() * hit.GetCharge();
            hitQ += hit.GetCharge();
         });
         x /= hitQ;
         y /= hitQ;
         hitCluster->SetCharge(hitQ);
         hitCluster->SetPosition(x, y, hitTBArray.at(0).GetPosition().Z());
         hitCluster->SetTimeStamp(hitTBArray.at(0).GetTimeStamp());
         TMatrixDSym cov(3); // TODO: Setting covariant matrix based on pad size and drift time resolution. Using
                             // estimations for the moment.
         cov(0, 1) = 0;
         cov(1, 2) = 0;
         cov(2, 0) = 0;
         cov(0, 0) = 0.04;
         cov(1, 1) = 0.04;
         cov(2, 2) = 0.01;
         hitCluster->SetCovMatrix(cov);
         ++clusterID;
         track.AddClusterHit(hitCluster);
      }
   }

   // Sanity check
   /*std::vector<AtHitCluster> *hitClusterArray = track.GetHitClusterArray();
   std::cout << " Clusterized hits : " << hitClusterArray->size() << "\n";
   for (auto iClusterHits = 0; iClusterHits < hitClusterArray->size(); ++iClusterHits) {
      TVector3 pos = hitClusterArray->at(iClusterHits).GetPosition();
      double clusterQ = hitClusterArray->at(iClusterHits).GetCharge();
      int TB = hitClusterArray->at(iClusterHits).GetTimeStamp();
      std::cout << " Pos : " << pos.X() << " - " << pos.Y() << " - " << pos.Z() << " - TB : " << TB
                << " - Charge : " << clusterQ << "\n";
      }*/
}
