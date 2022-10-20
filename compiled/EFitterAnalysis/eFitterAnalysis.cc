#include "eFitterAnalysis.h"

int main(int argc, char *argv[])
{

   // TODO Hardcoded parameters in fitter constructor
   // fEnableMerging = 1;
   // fEnableSingleVertexTrack = 1;
   // fExpNum = e20020;
  
   
  
  
   // Work directory
   TString dir = getenv("VMCWORKDIR");
   std::string dirCstr = dir.Data();

   // Geometry file
   TString geoManFile =
      dir + "/geometry/ATTPC_He1bar_v2_geomanager.root"; 
   // Ion list file
   std::string ionList = dirCstr + "/resources/ionFitLists/e20020_ionList.xml";

   // Analysis flow parameters
   std::size_t firstEvt = 0;
   std::size_t lastEvt = 0;
   bool fInteractiveMode = 1;
   TString inputFileName = "";
   bool fitDirection = 0; // 0: Forward (d,d) - 1: Backwards (d,p)
   bool simulationConv = 0;
   bool enableMerging = 1;
   bool enableSingleVertexTrack = 1;
   bool enableReclustering = 1;//For benchmarking purposes
   Double_t clusterRadius = 7.5;//mm
   Double_t clusterDistance   = 15.0;//mm
   Exp exp = e20009;
   

   // Physics parameters
   Float_t magneticField = 3.0;        // T
   Float_t gasMediumDensity = 0.1533;  //  0.1533 mg/cm3 (a,a) - 0.13129 mg/cm3 (d,p)
  
   
   
   //  Arguments
   if (argc == 7) {
      firstEvt = std::atoi(argv[1]);
      lastEvt = std::atoi(argv[2]);
      fInteractiveMode = std::atoi(argv[3]);
      inputFileName = argv[4];
      fitDirection = std::atoi(argv[5]);
      simulationConv = std::atoi(argv[6]);
      std::cout << cGREEN << " ------------------------------------------------- "
                << "\n";
      std::cout << " Processing file : " << inputFileName << "\n";
      std::cout << " Processing events from : " << firstEvt << " to " << lastEvt << "\n";
      std::cout << " Fit direction : " << fitDirection << "\n";
      std::cout << " Simulation ? " << simulationConv << "\n";
      std::cout << " Interactive mode ? : " << fInteractiveMode << "\n";
      std::cout << " Merging ? : " << enableMerging << "\n";
      std::cout << " Single Vertex Track ? " << cNORMAL << "\n";

   } else {
      std::cout << " Wrong number of arguments. Expecting 7: first_event last_event interactive_mode_bool "
                   "fileNameWithoutExtension fitDirection_bool simulation_convention."
                << "\n";
      return 0;
   }
     
   // File paths
   TString filePath;
   TString simFile;
   TString outputFileName;

   switch(exp)
      {
       case e20009:
	gasMediumDensity = 0.13129;
	
	if (simulationConv) {
         filePath = dir + "/macro/Simulation/ATTPC/10Be_dp/";
         simFile = "_sim_";
        } else {
         filePath = dir + "/macro/Unpack_HDF5/e20009/";
         simFile = "";
        }
	
        geoManFile = dir + "/geometry/ATTPC_D600torr_v2_geomanager.root";
        ionList = dirCstr + "/resources/ionFitLists/e20009_ionList.xml";

        std::cout << " Analysis of experiment e20009. Gas density : " << gasMediumDensity << " mg/cm3"
                  << "\n";
        std::cout << " File path : " << filePath << "\n";
        std::cout << " Geomtry file : " << geoManFile << "\n";
        std::cout << " Ion list file : " << ionList << "\n";

        break;

       case e20020:
	gasMediumDensity = 0.1533;

	if (simulationConv) {
         filePath = dir + "/macro/Simulation/ATTPC/16O_aa_v2/"; 
         simFile = "_sim_";
        } else {
         filePath = dir + "/macro/Unpack_HDF5/e20020/"; 
         simFile = "";
        }

	
        geoManFile = dir + "/geometry/ATTPC_He1bar_v2_geomanager.root";
        ionList = dirCstr + "/resources/ionFitLists/e20020_ionList.xml";

        std::cout << " Analysis of experiment e20020. Gas density : " << gasMediumDensity << " mg/cm3"
                  << "\n";
        std::cout << " File path : " << filePath << "\n";
        std::cout << " Geomtry file : " << geoManFile << "\n";
        std::cout << " Ion list file : " << ionList << "\n";
        break;
     }
   



   

   outputFileName = "fit_analysis_" + simFile + inputFileName;
   outputFileName += "_" + std::to_string(firstEvt) + "_" + std::to_string(lastEvt) + ".root";

   inputFileName = filePath + inputFileName + ".root";

   std::cout<<" Input file name : "<<inputFileName<<"\n";
   
   ////FitManager becomes owner

   std::shared_ptr<FitManager> fitManager;

   try {

      fitManager = std::make_shared<FitManager>(ionList);

   } catch (std::exception &ex) {
      std::cout << "Exception in FitManager constructor " << ex.what() << "!\n";
      std::exit(EXIT_FAILURE);
   }

   fitManager->SetGeometry(geoManFile, magneticField, gasMediumDensity);
   fitManager->SetInputFile(inputFileName, firstEvt, lastEvt);
   fitManager->SetOutputFile(outputFileName);
   fitManager->SetFitters(simulationConv);
   fitManager->SetFitDirection(fitDirection);
   fitManager->EnableMerging(enableMerging);
   fitManager->EnableSingleVertexTrack(enableSingleVertexTrack);
   fitManager->EnableReclustering(enableReclustering,clusterRadius,clusterDistance);
   fitManager->SetExpNum(exp);
   
   if (fInteractiveMode)
      fitManager->EnableGenfitDisplay();

   // FairRun
   FairRunAna *run = new FairRunAna();
   run->SetGeomFile(geoManFile.Data());

   // Kinematics tool
   std::shared_ptr<AtTools::AtKinematics> kine = std::make_shared<AtTools::AtKinematics>();

   auto reader = fitManager->GetReader();

   // Event loop
   for (auto i = firstEvt; i < lastEvt; i++) {

      std::cout << cGREEN << "  >>  Event Number : " << i << cNORMAL << "\n";
      fitManager->ClearTree();

      reader->Next();

      AtPatternEvent *patternEvent = fitManager->GetPatternEve();
      AtEvent *event = fitManager->GetEve();

      if (patternEvent) {

         auto &auxPadArray = event->GetAuxPads();
         std::cout << cGREEN << "   >>>> Number of auxiliary pads : " << auxPadArray.size() << cNORMAL << "\n";

         std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
         std::cout << cGREEN << "   >>>> Number of pattern tracks " << patternTrackCand.size() << cNORMAL << "\n";

         fitManager->GetAuxiliaryChannels(auxPadArray);

         fitManager->FitTracks(patternTrackCand);

      } // pattern event

      fitManager->FillTree();

   } // Event

   fitManager->ChangeToOutputFile();
   fitManager->WriteTree();
   fitManager->CloseOutputFile();

   genfit::EventDisplay *eveDisplay = fitManager->GetEventDisplay();

   if (eveDisplay) {
      // open event display
      eveDisplay->open();

   } // Interactive mode

   std::cout << " End of run "
             << "\n";

   return 0;
}

FitManager::FitManager() {}

FitManager::FitManager(std::string list)
{
   fVerbosity = 0;
   fSimulationConv = 0;
   fMagneticField = 0.0;
   fGasDensity = 0.0;
   fEnableMerging = 0;
   fFitDirection = 0;
   fEnableSingleVertexTrack = 0;

   fKinematics = std::make_shared<AtTools::AtKinematics>();
   fTrackTransformer = std::make_unique<AtTools::AtTrackTransformer>();

   // Parse list of ions to fit
   fParser.ParseIonFitXML(list);
   ionList = fParser.GetIonFile();
   fWorkDir = getenv("VMCWORKDIR");

   if (ionList->size() == 0) {
      std::cerr << cRED << " Error. Ion list is empty. Exiting..." << cNORMAL << "\n";
      throw std::runtime_error("Empty ion list");
   } else {
      std::cout << " "
                << "\n";
      std::cout << cGREEN << " List of ions to fit : " << cNORMAL << "\n";
      for (auto ion : *ionList) {
         std::cout << ion << "\n";
         std::ifstream elFile(fWorkDir.Data() + ion._eLossFile);
         if (!elFile.good()) {
            std::cout << cRED << " File " << ion._eLossFile << " not found. Exiting..." << cNORMAL << "\n";
            throw std::runtime_error("Could not open file");
         }
      }
   } // ion list
}

FitManager::~FitManager() {}

Bool_t FitManager::FitTracks(std::vector<AtTrack> &tracks)
{

   // TODO
   Double_t distThres = 150.0;

   // Merge tracks and calculate directions
   std::vector<AtTrack> mergedTrackPool;
   std::vector<AtTrack *> candTrackPool;
   Int_t eventMultiplicity = 0;
   Int_t praMultiplicity = 0;

   auto sp = std::unique_ptr<AtTrack[]>(new AtTrack[tracks.size()]);

   for (auto iTrack = 0; iTrack < tracks.size(); ++iTrack) {
      AtTrack track = tracks.at(iTrack);

      std::cout << cYELLOW << " Track " << track.GetTrackID() << " with " << track.GetHitClusterArray()->size()
                << " clusters and " << track.GetHitArray().size() << " hits. " << cNORMAL << "\n";

      trackID = track.GetTrackID();
      trackIDVec.push_back(trackID);

      if (track.GetHitClusterArray()->size() < 3) {
         std::cout << cRED << " Track has less than 3 clusters! " << cNORMAL << "\n";
         continue;
      }

      if (track.GetTrackID() == -1) {
         std::cout << cRED << " Track is noise! " << cNORMAL << "\n";
         continue;
      }

      ++praMultiplicity;

      // Track merging
      Double_t theta = track.GetGeoTheta();
      std::pair<Double_t, Double_t> center = track.GetGeoCenter();
      Double_t thetaConv;

      if (fSimulationConv)
         thetaConv = 180.0 - theta * TMath::RadToDeg();
      else
         thetaConv = theta * TMath::RadToDeg();

      auto hitClusterArray = track.GetHitClusterArray();
      AtHitCluster iniCluster;
      AtHitCluster secCluster;
      AtHitCluster endCluster;
      Double_t zIniCal = 0;
      Double_t zEndCal = 0;
      XYZPoint iniPos;
      XYZPoint secPos;
      XYZPoint endPos;

      if (thetaConv < 90.0) { // Forward tracks
         iniCluster =
            hitClusterArray->back(); // NB: Use back because We do not reverse the cluster vector like in AtGenfit!
         secCluster = hitClusterArray->at(hitClusterArray->size() - 2);
         iniPos = iniCluster.GetPosition();
         secPos = secCluster.GetPosition();
         endCluster = hitClusterArray->front();
         endPos = endCluster.GetPosition();
         zIniCal = 1000.0 - iniPos.Z();
         zEndCal = 1000.0 - endPos.Z();
      } else if (thetaConv > 90.0) { // Backward tracks
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

      // NB: Mind the x sign. Currently set for backward tracks
      // std::cout<<"   Old phi from PRA : "<<track.GetGeoPhi()*TMath::RadToDeg()<<"\n";
      Double_t phiClus = 0;

      if (thetaConv > 90) {
         phiClus = TMath::ATan2(secPos.Y() - iniPos.Y(), -secPos.X() + iniPos.X());
         track.SetGeoPhi(-phiClus);
      } else if (thetaConv < 90) {
         phiClus = TMath::ATan2(secPos.Y() - iniPos.Y(), -secPos.X() + iniPos.X());
         track.SetGeoPhi(phiClus);
      }

      // This is just to select distances
      std::cout << cBLUE << "   Track ID " << track.GetTrackID() << "\n";
      std::cout << "   Initial position : " << xiniPRA << " - " << yiniPRA << " - " << ziniPRA << " "
                << iniCluster.GetTimeStamp() << "\n";
      std::cout << "   Second position : " << secPos.X() << " - " << secPos.Y() << " - " << secPos.Z() << " "
                << secCluster.GetTimeStamp() << "\n";
      std::cout << "   End position : " << endPos.X() << " - " << endPos.Y() << " - " << zEndCal << " "
                << endCluster.GetTimeStamp() << "\n";
      std::cout << "   Theta (PRA) " <<track.GetGeoTheta() * TMath::RadToDeg()<<"   Theta (convention) : " << thetaConv << " - Phi Clus : " << phiClus * TMath::RadToDeg() << "\n";
      std::cout << "   Track center - X :  " << center.first << " - Y : " << center.second << "\n";
      std::cout << "   Track phi recalc : " << track.GetGeoPhi() * TMath::RadToDeg() << cNORMAL << "\n";

      // Skip tracks that are far from Z (to be checked against number of iterations for extrapolation)
      Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());
      std::cout << KRED << "    Distance to Z " << dist << cNORMAL << "\n";
      std::cout << KGRN << "    ---- Adding track candidate " << cNORMAL << "\n";
      track.SetVertexToZDist(dist);
      if (fEnableMerging) {
         sp[iTrack] = track;
         candTrackPool.push_back(std::move(&sp[iTrack]));

      } else {
         continue;
      }

   } // Tracks

   FindSingleTracks(candTrackPool);

   // Find candidate tracks closer to vertex and discard duplicated

   std::vector<AtTrack *> candToMergePool;

   if (fEnableMerging && !fSimulationConv) { // TODO: Not adapted to simulation yet

      for (auto itA = candTrackPool.begin(); itA != candTrackPool.end(); ++itA) {

         candToMergePool.clear();
         AtTrack *trA = *(itA);
         std::cout << " Processing track : " << trA->GetTrackID() << "\n";

         auto itB = std::copy_if(itA + 1, candTrackPool.end(), std::back_inserter(candToMergePool),
                                 [&trA, this](AtTrack *track) { return CompareTracks(trA, track); });

         if (candToMergePool.size() > 0) { // Merge if matches are found
            candToMergePool.push_back(trA);
            Bool_t merged = fFitter->MergeTracks(&candToMergePool, &mergedTrackPool, fEnableSingleVertexTrack,fClusterRadius,fClusterSize);//NB: Reclustering is also performed here

         } else {
            if (!trA->GetIsMerged() && trA->GetVertexToZDist() < distThres)
               mergedTrackPool.push_back(*trA);
         }

         // std::cout<<" Merged track pool size "<<mergedTrackPool.size()<<" - candidate to merge pool size
         // "<<candToMergePool.size()<<"\n";
      }

   } else {
      for (auto track : candTrackPool)
         mergedTrackPool.push_back(*track);
   }

   std::cout << "\n";
   std::cout << KGRN << "    Candidate/Merged Tracks Pool Size : " << mergedTrackPool.size() << "\n";
   std::cout << "    Been merged? " << fEnableMerging << "\n";
   std::cout << "    Tracks prepared. Proceeding with fits. " << cNORMAL << "\n";
   std::cout << "\n";

   eventMultiplicity = mergedTrackPool.size();

   // Fitting track candidates
   for (auto track : mergedTrackPool) {

      
      if (fEnableReclustering) {
           track.ResetHitClusterArray();
           fTrackTransformer->ClusterizeSmooth3D(track,fClusterRadius,fClusterSize); //NB: Just for analysis benchmarking
       }

      Double_t theta = track.GetGeoTheta();
      Double_t radius = track.GetGeoRadius() / 1000.0; // mm to m
      Double_t phi = track.GetGeoPhi();
      Double_t brho = fMagneticField * radius / TMath::Sin(theta); // Tm
      Double_t points = track.GetHitArray().size();

      std::cout << "      Merged track - Theta : " << theta * TMath::RadToDeg() << " Phi : " << phi * TMath::RadToDeg()
                << "\n";

      auto hitClusterArray = track.GetHitClusterArray();
      AtHitCluster iniCluster;
      Double_t zIniCal = 0;
      XYZPoint iniPos;

      // for(auto hitCluster : *hitClusterArray)
      // std::cout<<" Cluster hit "<<hitCluster.GetHitID()<<" - "<<hitCluster.GetPosition().X()<<" -
      // "<<hitCluster.GetPosition().Y()<<" - "<<1000.0-hitCluster.GetPosition().Z()<<"\n";

      // Variable for convention (simulation comes reversed)
      Double_t thetaConv;
      if (fSimulationConv) {
         thetaConv = 180.0 - theta * TMath::RadToDeg();
      } else {
         thetaConv = theta * TMath::RadToDeg();
      }

      if (thetaConv < 90.0) {
         iniCluster =
            hitClusterArray->back(); // NB: Use back because We do not reverse the cluster vector like in AtGenfit!
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
      std::cout << cGREEN << "      Merged track - Initial position : " << xiniPRA << " - " << yiniPRA << " - "
                << ziniPRA << cNORMAL << "\n";

      // Skip border angles
      //    if (theta * TMath::RadToDeg() < 5 || theta * TMath::RadToDeg() > 175)
      // continue;
      // Skip tracks that are far from Z (to be checked against number of iterations for extrapolation)
      Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());

      std::cout << KRED << "       Merged track - Distance to Z (Candidate Track Pool) " << dist << cNORMAL << "\n";
      
      
      // Fitters
      for (auto fitter : fFitters)
         fitter->Init();

      // Kinematic filters and fit selection
      
      std::vector<Int_t> pdgCandFit;
      if (thetaConv > 90) {

        switch(fExpNum){
	 case e20020:
	   pdgCandFit.push_back(1000010020);
	   break;
	 case e20009:
	   pdgCandFit.push_back(2212);
	   break;
	}
	
      } else if (thetaConv < 90 && thetaConv > 0) { 

       switch(fExpNum){
	 case e20020:
	   pdgCandFit.push_back(1000020040);
	   break;
	 case e20009:
	   pdgCandFit.push_back(1000010020);
	   break;
	}
	
      } else if (thetaConv < 0) {

         // continue;
         // pdgCandFit.push_back(1000040100);
         // pdgCandFit.push_back(1000040110);
      }

      try {

         for (auto pdg : pdgCandFit) {

            genfit::Track *fitTrack;

            auto fIt = std::find_if(std::begin(fFitters), std::end(fFitters), [&pdg](AtFITTER::AtFitter *fitter) {
               return dynamic_cast<AtFITTER::AtGenfit *>(fitter)->GetPDGCode() == pdg;
            });
            if (fIt != fFitters.end()) {
               int index = std::distance(fFitters.begin(), fIt);
               std::cout << cBLUE << "  -  Fitter for : " << pdg << " found in " << index << cNORMAL << "\n";
               fitTrack = fFitters[index]->FitTracks(&track);
            } else {

               std::cout << cRED << " Error! Fitter not found for : " << pdg << "\n";
               std::exit(EXIT_FAILURE);
            }

            Int_t atomicNumber = 0;
            Int_t massNumber = 0;
            Double_t M_Ener = 0.0;

            auto fIl = std::find_if(ionList->begin(), ionList->end(),
                                    [&pdg](AtTools::IonFitInfo ion) { return ion._PDG == pdg; });
            if (fIl != ionList->end()) {

               int index = std::distance(ionList->begin(), fIl);
               std::cout << cBLUE << "  -  Ion info for : " << pdg << " found in " << index << cNORMAL << "\n";
               atomicNumber = ionList->at(index)._atomicNumber;
               massNumber = ionList->at(index)._MassNumber;
               M_Ener = massNumber * 931.49401 / 1000.0;
            }

            // Kinematics from PRA

            std::tuple<Double_t, Double_t> mom_ener = fKinematics->GetMomFromBrho(massNumber, atomicNumber, brho);
            EPRA = std::get<1>(mom_ener) * 1000.0;
            APRA = theta * TMath::RadToDeg();
            PhiPRA = phi * TMath::RadToDeg();

            // Extract info from Fit track

            pdgVec.push_back(std::to_string(pdg));

            TVector3 pos_res;
            TVector3 mom_res;
            TMatrixDSym cov_res;

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

            // Fit convergence
            Int_t fitConverged = 0;

            // Reset variables assigned in fitting
            pVal = -1;
            trackLength = 0;
            xiniFitXtr = -1000.0;
            yiniFitXtr = -1000.0;
            ziniFitXtr = -1E4;
            xiniFit = -1000.0;
            yiniFit = -1000.0;
            ziniFit = -1E4;
            POCAXtr = -1000.0;
            EFit = -10.0;
            EFitXtr = -10.0;
            AFit = 0.0;
            PhiFit = 0.0;
            particleQ = -10.0;

            // PID
            Double_t len = 0;
            Double_t eloss = 0;
            Double_t dedx = 0;

            // Energy loss from ADC
            auto hitClusterArray = track.GetHitClusterArray();
            std::size_t cnt = 0;

            if (thetaConv < 90) {

               auto it = hitClusterArray->rbegin();
               while (it != hitClusterArray->rend()) {

                  if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.5)
                     break;
                  auto dir = (*it).GetPosition() - (*std::next(it, 1)).GetPosition();
                  eloss += (*it).GetCharge();
                  len += std::sqrt(dir.Mag2());
                  dedx += (*it).GetCharge();
                  // std::cout<<(*it).GetCharge()<<"\n";
                  it++;
                  ++cnt;
               }
            } else if (thetaConv > 90) {

               eloss += hitClusterArray->at(0).GetCharge();

               cnt = 1;
               for (auto iHitClus = 1; iHitClus < hitClusterArray->size(); ++iHitClus) {

                  if (((Float_t)cnt / (Float_t)hitClusterArray->size()) > 0.5)
                     break;
                  auto dir =
                     hitClusterArray->at(iHitClus).GetPosition() - hitClusterArray->at(iHitClus - 1).GetPosition();
                  len += std::sqrt(dir.Mag2());
                  eloss += hitClusterArray->at(iHitClus).GetCharge();
                  dedx += hitClusterArray->at(iHitClus).GetCharge();
                  // std::cout<<len<<" - "<<eloss<<" - "<<hitClusterArray->at(iHitClus).GetCharge()<<"\n";
                  ++cnt;
               }
            }

            eloss /= cnt;
            dedx /= len;

            if (fitTrack == nullptr)
               continue;

            try {

               if (fitTrack && fitTrack->hasKalmanFitStatus()) {

                  auto KalmanFitStatus = fitTrack->getKalmanFitStatus();
                  auto trackRep = fitTrack->getTrackRep(0); // Only one representation is sved for the moment.
                  fitConverged = KalmanFitStatus->isFitConverged(false);

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
                     mom_ext = fitState.getMom();
                     pos_ext = fitState.getPos();

                     // Backward extrapolation
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
                           } else {

                              break;
                           }

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
                     if (display)
                        display->addEvent(fitTrack);

                     Double_t thetaA = 0.0;
                     if (thetaConv > 90.0) {
                        thetaA = 180.0 * TMath::DegToRad() - mom_res.Theta();

                     } else {
                        thetaA = mom_res.Theta();
                     }

                     Double_t E = TMath::Sqrt(TMath::Power(mom_res.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener;
                     EFit = E * 1000.0;
                     EFitXtr =
                        1000.0 * (TMath::Sqrt(TMath::Power(mom_ext.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener);
                     std::cout << " Energy : " << E * 1000.0 << " - Energy Xtr : " << EFitXtr << "\n";
                     AFit = thetaA * TMath::RadToDeg();
                     PhiFit = mom_res.Phi();
                     xiniFit = pos_res.X();
                     yiniFit = pos_res.Y();
                     ziniFit = pos_res.Z();

                  } // Kalman fit

               } // Kalman status

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
            brhoVec.push_back(brho);
            trackPointsVec.push_back(points);
            fitConvergedVec.push_back(fitConverged);

            dEdxADC.push_back(dedx);
            eLossADC.push_back(eloss);

            evMult = eventMultiplicity;
            praMult = praMultiplicity;

         } // pdg cand

      } catch (std::exception &e) {
         std::cout << " Exception fitting track !" << e.what() << "\n";
         continue;
      }

   } // Merged track loop

   return 0;
}

Bool_t FitManager::SetFitters(Bool_t simConv)
{
   fSimulationConv = simConv;

   for (auto ion : *ionList) {
     std::cout << " Creating fitter for : " << ion._ionName << " - " << (Int_t)ion._PDG << "\n";
     std::cout << " Energy loss file : " << fWorkDir.Data() + ion._eLossFile << "\n";
     fFitter = new AtFITTER::AtGenfit(fMagneticField, 0.00001, 1000.0, fWorkDir.Data() + ion._eLossFile, fGasDensity,
                                      (Int_t)ion._PDG);
     // dynamic_cast<AtFITTER::AtGenfit*>(fFitter)->SetPDGCode((Int_t)ion._PDG);
     dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetIonName(ion._ionName);
     dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetMass((Double_t)ion._mass);
     dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetAtomicNumber((Int_t)ion._atomicNumber);
     dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetNumFitPoints(1.0);
     dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetVerbosityLevel(1);
     dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetSimulationConvention(fSimulationConv);
     fFitters.push_back(fFitter);
   }
   return true;
}

Bool_t FitManager::SetGeometry(TString file, Float_t field, Float_t density)
{
   std::cout << " Geometry file : " << file.Data() << "\n";
   new TGeoManager("Geometry", "ATTPC geometry");
   TGeoManager::Import(file.Data());
   genfit::MaterialEffects::getInstance()->init(new genfit::TGeoMaterialInterface());
   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0., field * 10.0)); //
   fMagneticField = field;
   fGasDensity = density;
   return true;
}

Bool_t FitManager::EnableGenfitDisplay()
{

   display = genfit::EventDisplay::getInstance();
   return true;
}

Bool_t FitManager::SetInputFile(TString &file, std::size_t firstEve, std::size_t lastEve)
{
   Int_t nEvents = lastEve - firstEve;
   std::cout << " Opening File : " << file.Data() << "\n";
   std::cout << " Number of events : " << nEvents << std::endl;

   fInputFile = std::make_shared<TFile>(file.Data(), "READ");
   fReader = std::make_shared<TTreeReader>("cbmsim", fInputFile.get());
   fPatternEveArray = std::make_shared<TTreeReaderValue<TClonesArray>>(*fReader, "AtPatternEvent");
   fEveArray = std::make_shared<TTreeReaderValue<TClonesArray>>(*fReader, "AtEventH");
   fReader->SetEntriesRange(firstEve, lastEve);

   return true;
}

void FitManager::GetAuxiliaryChannels(const std::vector<AtAuxPad> &padArray)
{
   for (const auto& pad : padArray) {

      if (pad.GetAuxName().compare(std::string("IC_sca")) == 0) {

         auto adc = pad.GetADC();
         ICMult = GetNPeaksHRS(&ICTimeVec, &ICVec, adc.data());
      }
      if (pad.GetAuxName().compare(std::string("IC")) == 0) {
         auto& adc = pad.GetADC();

         for (auto iadc = 0; iadc < 512; ++iadc)
            ICEVec.push_back(adc[iadc]);
      }
   }
}

Double_t FitManager::GetNPeaksHRS(std::vector<Int_t> *timeMax, std::vector<Float_t> *adcMax, double *adc_test)
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
Double_t FitManager::GetMaximum(double *adc)
{
   double max = 0;

   for (int indTB = 0; indTB < 512; ++indTB) {
      // std::cout<<" TB "<<indTB<<" adc "<<adc[indTB]<<"\n";
      if (adc[indTB] > max)
         max = adc[indTB];
   }

   return max;
}

firstOrbit FitManager::GetFirstOrbit(genfit::Track *track, genfit::AbsTrackRep *rep, TVector3 vertex)
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

void FitManager::ConstructTrack(const genfit::StateOnPlane *prevState, const genfit::StateOnPlane *state,
                                const genfit::AbsTrackRep *rep, std::vector<TVector3> &track,
                                std::vector<trackSegment> &segments)
{

   if (prevState == nullptr || state == nullptr) {
      // std::cerr << "prevState == nullptr || state == nullptr\n";
      return;
   }

   int pdg = rep->getPDG();

   double massAMU = 1.007276466812;

   //TODO Temporary solution
   if(pdg == 1000020040)
     massAMU = 4.0026;
   else if(pdg == 2212)
     massAMU = 1.00728;
   else if(pdg == 1000010020)
     massAMU = 2.01355;
   else if(pdg == 1000060120)
     massAMU = 12;
   else if(pdg == 1000080160)
     massAMU = 15.9949;
   else
     {
       std::cerr<<" FitManager::ConstructTrack - Error! PDG code not found. Exiting..."<<"\n";
       std::exit(0);
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
   // TODO
   Double_t mass = massAMU * 931.49401 / 1000.0;
   Double_t ELoss2 = 1000.0 * (TMath::Sqrt(TMath::Power((mom).Mag(), 2) + TMath::Power(mass, 2)) - mass);
   Double_t ELoss1 = 1000.0 * (TMath::Sqrt(TMath::Power((oldMom).Mag(), 2) + TMath::Power(mass, 2)) - mass);
   Double_t ELoss = ELoss1 - ELoss2;

   trackSegment segment = {ELoss, pos, dirToMom, dir, dirToMom.Theta(), dirToMom.Phi(), (UInt_t)segments.size()};
   segments.push_back(segment);

   track.push_back(oldPos);
   track.push_back(intermediate1);
   track.push_back(intermediate2);
}

Bool_t FitManager::CompareTracks(AtTrack *trA, AtTrack *trB)
{

   // Matching angle
   // Matching center
   // Vertex position
   // Track overlap

   // TODO
   Double_t angleSpread = 5.0 * TMath::DegToRad();
   Double_t distThresh = 100.0;
   Double_t distMax = 150.0;

   if (trA->GetIsMerged() || trB->GetIsMerged()) {
      // std::cout << "    Skipped : merged " << std::endl;
      return false;
   }
   if (trA->GetTrackID() == trB->GetTrackID()) {
      // std::cout << "    Skipped : Same track " << std::endl;
      return false;
   }
   if (trA->GetGeoTheta() * TMath::RadToDeg() > 90 && trB->GetGeoTheta() * TMath::RadToDeg() < 90) {
      // std::cout << "    Skipped : Diff directions " << std::endl;
      return false;
   }
   if (trA->GetGeoTheta() * TMath::RadToDeg() < 90 && trB->GetGeoTheta() * TMath::RadToDeg() > 90) {
      // std::cout << "    Skipped : Diff directions " << std::endl;
      return false;
   }

   std::cout << " Comparing track A : " << trA->GetTrackID() << " and track B : " << trB->GetTrackID() << "\n";

   if ((trB->GetGeoTheta() - angleSpread) <= trA->GetGeoTheta() &&
       trA->GetGeoTheta() <= (trB->GetGeoTheta() + angleSpread)) {

      Double_t centerDistance = CenterDistance(trA, trB);

      if (centerDistance < distThresh) {

         if (!CheckOverlap(trA, trB)) {

            Double_t iniA = 0.0;
            Double_t endA = 0.0;
            Double_t iniB = 0.0;
            Double_t endB = 0.0;
            Double_t dist = 0.0;  // 3D distance
            Double_t distR = 0.0; // Radial distance
            AtHitCluster iniClusterA;
            AtHitCluster iniClusterB;
            AtHitCluster endClusterA;
            AtHitCluster endClusterB;

            // Find distance between end points of the tracks assuming no overlap
            if (trA->GetGeoTheta() * TMath::RadToDeg() < 90) {
               iniClusterA = trA->GetHitClusterArray()->back();
               iniClusterB = trB->GetHitClusterArray()->back();
               endClusterA = trA->GetHitClusterArray()->front();
               endClusterB = trB->GetHitClusterArray()->front();
               iniA = 1000.0 - iniClusterA.GetPosition().Z();
               iniB = 1000.0 - iniClusterB.GetPosition().Z();
               endA = 1000.0 - endClusterA.GetPosition().Z();
               endB = 1000.0 - endClusterB.GetPosition().Z();

            } else if (trA->GetGeoTheta() * TMath::RadToDeg() > 90) {
               iniClusterA = trA->GetHitClusterArray()->front();
               iniClusterB = trB->GetHitClusterArray()->front();
               endClusterA = trA->GetHitClusterArray()->back();
               endClusterB = trB->GetHitClusterArray()->back();
               iniA = iniClusterA.GetPosition().Z();
               iniB = iniClusterB.GetPosition().Z();
               endA = endClusterA.GetPosition().Z();
               endB = endClusterB.GetPosition().Z();
            }
            // std::cout<<" Track A Ini : "<<iniA<<"\n";
            // std::cout<<" Track B Ini : "<<iniB<<"\n";
            // std::cout<<" Track A End : "<<endA<<"\n";
            // std::cout<<" Track B End : "<<endB<<"\n";

            if (iniA < iniB) {
               dist = std::sqrt((endClusterA.GetPosition() - iniClusterB.GetPosition()).Mag2());
               distR = TMath::Sqrt(TMath::Power(endClusterA.GetPosition().X() - iniClusterB.GetPosition().X(), 2) +
                                   TMath::Power(endClusterA.GetPosition().Y() - iniClusterB.GetPosition().Y(), 2));
            } else {
               dist = std::sqrt((endClusterB.GetPosition() - iniClusterA.GetPosition()).Mag2());
               distR = TMath::Sqrt(TMath::Power(iniClusterA.GetPosition().X() - endClusterB.GetPosition().X(), 2) +
                                   TMath::Power(iniClusterA.GetPosition().Y() - endClusterB.GetPosition().Y(), 2));
            }

            // std::cout<<" -- Distance between tracks "<<dist<<"\n";
            // std::cout<<" -- Radial distance between tracks "<<distR<<"\n";

            std::cout << " --- Tracks valid for merging! - " << trA->GetTrackID() << " - " << trB->GetTrackID() << "\n";
            return true;

         } else
            std::cout << " Track Overlap found "
                      << " - " << trA->GetTrackID() << " - " << trB->GetTrackID() << "\n";
         return false;
      }

   } // Conditions
   return false;
}

Double_t FitManager::CenterDistance(AtTrack *trA, AtTrack *trB)
{
   std::pair<Double_t, Double_t> centerA = trA->GetGeoCenter();
   std::pair<Double_t, Double_t> centerB = trB->GetGeoCenter();
   std::cout << " Center A : " << centerA.first << " - " << centerA.second << "\n";
   std::cout << " Center B : " << centerB.first << " - " << centerB.second << "\n";
   Double_t centerDistance =
      TMath::Sqrt(TMath::Power(centerA.first - centerB.first, 2) + TMath::Power(centerA.second - centerB.second, 2));
   std::cout << " Center Distance : " << centerDistance << "\n";
   return centerDistance;
}
Bool_t FitManager::CheckOverlap(AtTrack *trA, AtTrack *trB)
{
   auto &hitArrayA = trA->GetHitArray();
   auto &hitArrayB = trB->GetHitArray();

   Int_t iTBMatch = 0;

   for (auto itA = hitArrayA.begin(); itA != hitArrayA.end(); ++itA) {

      std::vector<Int_t> iTBMatches;
      auto itB = hitArrayB.begin();
      while ((itB = std::find_if(itB, hitArrayB.end(), [&itA](AtHit &hitB) {
                 return hitB.GetTimeStamp() == itA->GetTimeStamp();
              })) != hitArrayB.end()) {
         iTBMatches.push_back(std::distance(hitArrayB.begin(), itB));
         itB++;
      }

      /*if(iTBMatches.size()>0){
      std::cout<<" TB Matches found for track A TB: "<<itA->GetTimeStamp()<<" at position
     "<<std::distance(hitArrayA.begin(), itA)<<"\n";

         for(auto itb =0;itb<iTBMatches.size();++itb)
     {
       std::cout<<" Index : "<<itb<<" - TB : "<<hitArrayB.at(itb).GetTimeStamp()<<"\n";
     }
     }*/

      iTBMatch += iTBMatches.size();
   }

   Double_t shortStraw = (hitArrayA.size() < hitArrayB.size()) ? hitArrayA.size() : hitArrayB.size();
   // TODO: % of overlap. Counted twice!
   // std::cout<<" Overlap "<<shortStraw<<" "<<iTBMatch<<"\n";
   if (iTBMatch > (shortStraw * 0.1))
      return true;
   else
      return false;
}

Bool_t FitManager::CheckAngles(AtTrack *trA, AtTrack *trB)
{
   // TODO
   Double_t angleSpread = 5.0 * TMath::DegToRad();

   if ((trB->GetGeoTheta() - angleSpread) <= trA->GetGeoTheta() &&
       trA->GetGeoTheta() <= (trB->GetGeoTheta() + angleSpread))
      return true;
   else
      return false;
}

std::vector<AtTrack *> FitManager::FindSingleTracks(std::vector<AtTrack *> &tracks)
{

   // TODO
   Double_t angleSpread = 5.0 * TMath::DegToRad();
   Double_t distThresh = 100.0;
   Double_t distMax = 100.0;
   std::vector<AtTrack *> singleTracks;
   std::vector<AtTrack *> fragmentedTracks;

   for (auto itA = tracks.begin(); itA != tracks.end(); ++itA) {
      AtTrack *trA = *(itA);
      std::cout << " Checking if track " << trA->GetTrackID() << " is single."
                << "\n";
      Bool_t isSingle = false;
      // Length - center - overlap
      for (auto itB = itA + 1; itB != tracks.end(); ++itB) {
         AtTrack *trB = *(itB);
         std::cout << " Track A : " << trA->GetTrackID() << " - Track B : " << trB->GetTrackID() << "\n";
         std::cout << " Track A Theta : " << trA->GetGeoTheta() * TMath::RadToDeg()
                   << " - Track B : " << trB->GetGeoTheta() * TMath::RadToDeg() << "\n";
         std::cout << " Track A Phi : " << trA->GetGeoPhi() * TMath::RadToDeg()
                   << " - Track B : " << trB->GetGeoPhi() * TMath::RadToDeg() << "\n";

         Double_t centerDistance = CenterDistance(trA, trB);
      }
   }

   return singleTracks;
}

Bool_t FitManager::SetOutputFile(TString &file)
{

   fOutputFile = std::make_shared<TFile>(file.Data(), "RECREATE");
   fOutputTree = std::make_shared<TTree>("outputTree", "OutputTree");

   fOutputTree->Branch("EFit", &EFit, "EFit/F");
   fOutputTree->Branch("AFit", &AFit, "AFit/F");
   fOutputTree->Branch("PhiFit", &PhiFit, "PhiFit/F");
   fOutputTree->Branch("EPRA", &EPRA, "EPRA/F");
   fOutputTree->Branch("APRA", &APRA, "APRA/F");
   fOutputTree->Branch("PhiPRA", &PhiPRA, "PhiPRA/F");
   fOutputTree->Branch("Ex", &Ex, "Ex/F");
   fOutputTree->Branch("ExXtr", &ExXtr, "ExXtr/F");
   fOutputTree->Branch("xiniFit", &xiniFit, "xiniFit/F");
   fOutputTree->Branch("yiniFit", &yiniFit, "yiniFit/F");
   fOutputTree->Branch("ziniFit", &ziniFit, "ziniFit/F");
   fOutputTree->Branch("xiniPRA", &xiniPRA, "xiniPRA/F");
   fOutputTree->Branch("yiniPRA", &yiniPRA, "yiniPRA/F");
   fOutputTree->Branch("ziniPRA", &ziniPRA, "ziniPRA/F");
   fOutputTree->Branch("EFitXtr", &EFitXtr, "EFitXtr/F");
   fOutputTree->Branch("xiniFitXtr", &xiniFitXtr, "xiniFitXtr/F");
   fOutputTree->Branch("yiniFitXtr", &yiniFitXtr, "yiniFitXtr/F");
   fOutputTree->Branch("ziniFitXtr", &ziniFitXtr, "ziniFitXtr/F");
   fOutputTree->Branch("distXtr", &distXtr, "distXtr/F");
   fOutputTree->Branch("pVal", &pVal, "pVal/F");
   fOutputTree->Branch("IC", &IC, "IC/F");
   fOutputTree->Branch("ICMult", &ICMult, "ICMult/I");
   fOutputTree->Branch("trackLength", &trackLength, "trackLength/F");
   fOutputTree->Branch("POCAXtr", &POCAXtr, "POCAXtr/F");
   fOutputTree->Branch("trackID", &trackID, "trackID/F");
   fOutputTree->Branch("evMult", &evMult, "evMult/I");
   fOutputTree->Branch("praMult", &praMult, "praMult/I");

   fOutputTree->Branch("EFitVec", &EFitVec);
   fOutputTree->Branch("AFitVec", &AFitVec);
   fOutputTree->Branch("PhiFitVec", &PhiFitVec);
   fOutputTree->Branch("EPRAVec", &EPRAVec);
   fOutputTree->Branch("APRAVec", &APRAVec);
   fOutputTree->Branch("PhiPRAVec", &PhiPRAVec);
   fOutputTree->Branch("ExVec", &ExVec);
   fOutputTree->Branch("ExXtrVec", &ExXtrVec);
   fOutputTree->Branch("xiniFitVec", &xiniFitVec);
   fOutputTree->Branch("yiniFitVec", &yiniFitVec);
   fOutputTree->Branch("ziniFitVec", &ziniFitVec);
   fOutputTree->Branch("xiniPRAVec", &xiniPRAVec);
   fOutputTree->Branch("yiniPRAVec", &yiniPRAVec);
   fOutputTree->Branch("ziniPRAVec", &ziniPRAVec);
   fOutputTree->Branch("EFitXtrVec", &EFitXtrVec);
   fOutputTree->Branch("xiniFitXtrVec", &xiniFitXtrVec);
   fOutputTree->Branch("yiniFitXtrVec", &yiniFitXtrVec);
   fOutputTree->Branch("ziniFitXtrVec", &ziniFitXtrVec);
   fOutputTree->Branch("distXtrVec", &distXtrVec);
   fOutputTree->Branch("pValVec", &pValVec);
   fOutputTree->Branch("ICVec", &ICVec);
   fOutputTree->Branch("ICTimeVec", &ICTimeVec);
   fOutputTree->Branch("trackLengthVec", &trackLengthVec);
   fOutputTree->Branch("POCAXtrVec", &POCAXtrVec);
   fOutputTree->Branch("trackIDVec", &trackIDVec);
   fOutputTree->Branch("fChi2Vec", &fChi2Vec);
   fOutputTree->Branch("bChi2Vec", &bChi2Vec);
   fOutputTree->Branch("fNdfVec", &fNdfVec);
   fOutputTree->Branch("bNdfVec", &bNdfVec);
   fOutputTree->Branch("ICEVec", &ICEVec);
   fOutputTree->Branch("particleQVec", &particleQVec);
   fOutputTree->Branch("POCAOrbZVec", &POCAOrbZVec);
   fOutputTree->Branch("firstOrbZVec", &firstOrbZVec);
   fOutputTree->Branch("phiOrbZVec", &phiOrbZVec);
   fOutputTree->Branch("lengthOrbZVec", &lengthOrbZVec);
   fOutputTree->Branch("eLossOrbZVec", &eLossOrbZVec);
   fOutputTree->Branch("brhoVec", &brhoVec);
   fOutputTree->Branch("eLossADC", &eLossADC);
   fOutputTree->Branch("dEdxADC", &dEdxADC);
   fOutputTree->Branch("pdgVec", &pdgVec);
   fOutputTree->Branch("trackPointsVec",&trackPointsVec);
   fOutputTree->Branch("fitConvergedVec", &fitConvergedVec);
   return true;
}

void FitManager::ClearTree()
{
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
   evMult = 0;
   praMult = 0;

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
   brhoVec.clear();
   eLossADC.clear();
   dEdxADC.clear();
   pdgVec.clear();
   trackPointsVec.clear();
   fitConvergedVec.clear();
}
