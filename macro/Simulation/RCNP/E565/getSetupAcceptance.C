
TGraph* ReadKinematics(TString kineFile);

void getSetupAcceptance()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   // Histogram definitions.
   TH2F *histVertexZvTrackThetaLABTotalSimulation = new TH2F("histVertexZvTrackThetaLABTotalSimulation", "histVertexZvTrackThetaLABTotalSimulation", 100, 0, 1000, 180, 0, 180);
   TH2F *histTrackKinematicsTotalSimulation = new TH2F("histTrackKinematicsTotalSimulation", "histTrackKinematicsTotalSimulation", 180, 0, 180, 80, 0, 20);

   TH2F *histVertexZvTrackThetaLABSiArray = new TH2F("histVertexZvTrackThetaLABSiArray", "histVertexZvTrackThetaLABSiArray", 100, 0, 1000, 180, 0, 180);
   TH2F *histTrackKinematicsSiArray = new TH2F("histTrackKinematicsSiArray", "histTrackKinematicsSiArray", 180, 0, 180, 80, 0, 20);

   TH2F *histVertexZvTrackThetaLABATTPC = new TH2F("histVertexZvTrackThetaLABATTPC", "histVertexZvTrackThetaLABATTPC", 100, 0, 1000, 180, 0, 180);
   TH2F *histTrackKinematicsATTPC = new TH2F("histTrackKinematicsATTPC", "histTrackKinematicsATTPC", 180, 0, 180, 80, 0, 20);

   TH2F *histVertexZvTrackThetaLABTotalAcceptance = new TH2F("histVertexZvTrackThetaLABTotalAcceptance", "histVertexZvTrackThetaLABTotalAcceptance", 100, 0, 1000, 180, 0, 180);
   TH2F *histTrackKinematicsTotalAcceptance = new TH2F("histTrackKinematicsTotalAcceptance", "histTrackKinematicsTotalAcceptance", 180, 0, 180, 80, 0, 20);

   TH1F *histChi2 = new TH1F("histChi2", "histChi2", 100, 0, 1000);

   TH1F *histHeavyELoss = new TH1F("histHeavyELoss", "histHeavyELoss", 100, 0, 1000);

   // Open the TCutFiles that may be needed.
   TFile *fileKinematicCuts = new TFile("./TCutFiles/kinematicsTCuts.root", "READ");
   TCutG *cutArtifactKinematics = (TCutG *)fileKinematicCuts->Get("cutArtifactKinematics");
   TCutG *cutKinematics = (TCutG *)fileKinematicCuts->Get("cutKinematics");
   fileKinematicCuts->Close();

   // Min and Max angles of the simulation.
   Double_t ThetaMinCMS = 0.0;
   Double_t ThetaMaxCMS = 40.0;

   // Open the digitalization file and get the TTree.
   TString digiFileName = TString::Format("/data/ATTPCROOTv2_results/E565/Simulation/digiFiles/output_digi_rcnp_13Be_p_%.1f_%.1f_600Torr_9mmBinning.root", ThetaMinCMS, ThetaMaxCMS);
   TFile *digiFile = new TFile(digiFileName, "READ");
   TTree *digiTree = (TTree *)digiFile->Get("cbmsim");
   int nDigiEvents = digiTree->GetEntries();
   std::cout << " Number of reconstructed events : " << double(nDigiEvents) / 2 << std::endl;

   // Open the MC file and get the TTree.
   TString mcFileName = TString::Format("/data/ATTPCROOTv2_results/E565/Simulation/simFiles/attpcsim_13Be_p_%.1f_%.1f_600Torr.root", ThetaMinCMS, ThetaMaxCMS);
   TFile *mcFile = new TFile(mcFileName, "READ");
   TTree *mcTree = (TTree *)mcFile->Get("cbmsim");
   int nMcEvents = mcTree->GetEntries();
   std::cout << " Number of simulated events : " << double(nMcEvents) / 2 << std::endl;

   // Creare the TTreeReader to read the AtTrackingEvents and simulation.
   TTreeReader digiReader("cbmsim", digiFile);
   TTreeReaderValue<TClonesArray> trackingArray(digiReader, "AtTrackingEvent");

   TTreeReader mcReader("cbmsim", mcFile);
   TTreeReaderValue<TClonesArray> mcTrackArray(mcReader, "MCTrack");
   TTreeReaderValue<TClonesArray> mcPointArray(mcReader, "AtTpcPoint");
   TTreeReaderValue<TClonesArray> mcSiPointArray(mcReader, "AtSiArrayPoint");

   // Text files where to save certain event numbers based on cuts.
   std::ofstream eventsArtifactKinematicsFile;
   eventsArtifactKinematicsFile.open("./filteredEventNumbers/eventsArtifactKinematics.txt");

   std::ofstream eventsKinematicsFile;
   eventsKinematicsFile.open("./filteredEventNumbers/eventsKinematics.txt");

   std::ofstream eventsChi2_1File;
   eventsChi2_1File.open("./filteredEventNumbers/eventsChi2_1.txt");

   std::ofstream eventsChi2_2File;
   eventsChi2_2File.open("./filteredEventNumbers/eventsChi2_2.txt");

   std::ofstream eventsChi2_3File;
   eventsChi2_3File.open("./filteredEventNumbers/eventsChi2_3.txt");

   // Number of fails.
   int nPunchThrough{};
   int nNotReconstructedELoss{};

   // Current event real MC vertex.
   double mcVertexZ{0};

   // Loop over events.
   for (int i = 0; i < nDigiEvents; i++) {
      digiReader.Next();
      mcReader.Next();

      // If i even, beam event. Get MC vertex and skip.
      if (i % 2 == 0) {
         int nMCPoints = mcPointArray->GetEntries();
         AtMCPoint *reactionPoint = (AtMCPoint *)mcPointArray->At(nMCPoints - 1);
         mcVertexZ = 1000 - reactionPoint->GetZ() * 10;
         continue;
      }

      // If there are not exactly 3 tracks in the drift region, skip entry. (We want proton, 12Be and neutron)
      if (mcTrackArray->GetEntries() != 3)
         continue;

      // Checking the total ELoss of the heavy residue in the drift region.
      double heavyTotalELoss{};
      for (int j = 0; j < mcPointArray->GetEntries(); j++) {
         AtMCPoint *mcPoint = (AtMCPoint *)mcPointArray->At(j);
         if (mcPoint->GetTrackID() == 2)
            heavyTotalELoss += mcPoint->GetEnergyLoss();
      }

      histHeavyELoss->Fill(heavyTotalELoss * 1000);

      // First of all, we check if there was a succesful measurement in the Si array detector.
      int nSi = mcSiPointArray->GetEntries();
      int nSi1{0};
      double ELossSi1{0};
      int nSi2{0};
      double ELossSi2{0};
      for (int idxSi = 0; idxSi < nSi; idxSi++) {
         AtMCPoint *mcPointSi = (AtMCPoint *) mcSiPointArray->At(idxSi);
         TString volName = mcPointSi->GetVolName();
         int trackID = mcPointSi->GetTrackID();
         if (trackID == 2 && volName.Contains("silicon1")) {
            ELossSi1 += mcPointSi->GetEnergyLoss();
            nSi1++;
            continue;
         }

         if (trackID == 2 && volName.Contains("silicon2")) {
            ELossSi2 += mcPointSi->GetEnergyLoss();
            nSi2++;
         }
      }

      // If we had a good Si array detector measurement, fill the Si acceptance histograms. Also keep track of this condition for later coincidence acceptance.
      AtMCTrack *mcTrackBeamlike = (AtMCTrack *)mcTrackArray->At(1);
      double kineticEnergyBeamlike = (mcTrackBeamlike->GetEnergy() - 13.03394 * 0.93149401) * 1000;
      double thetaBeamlike = 180 - TMath::ASin(mcTrackBeamlike->GetPt() / mcTrackBeamlike->GetP()) * TMath::RadToDeg();

      AtMCTrack *mcTrackScattered = (AtMCTrack *)mcTrackArray->At(0);
      double kineticEnergyScattered = (mcTrackScattered->GetEnergy() - mcTrackScattered->GetMass()) * 1000;
      double thetaScattered = 180 - TMath::ASin(mcTrackScattered->GetPt() / mcTrackScattered->GetP()) * TMath::RadToDeg();

      bool goodSiMeasurement{false};
      if (nSi1 && nSi2) {
         goodSiMeasurement = true;

         histVertexZvTrackThetaLABSiArray->Fill(mcVertexZ, thetaScattered);
         histTrackKinematicsSiArray->Fill(thetaScattered, kineticEnergyScattered);
      }

      // In any case, fill the total simulation histograms.
      histVertexZvTrackThetaLABTotalSimulation->Fill(mcVertexZ, thetaScattered);
      histTrackKinematicsTotalSimulation->Fill(thetaScattered, kineticEnergyScattered);

      // Now we take a look into the reconstructed AtTrackingEvent.
      AtTrackingEvent *trackingEvent = (AtTrackingEvent *)trackingArray->At(0);
      if (!trackingEvent)
         continue;

      auto &fittedTracks = trackingEvent->GetFittedTracks();

      int trackNum{};
      for (auto &fittedTrack: fittedTracks) {
         trackNum++;

         // Extract the metadata for this fit.
         auto &fitTrackMetadata = fittedTrack->GetTrackMetadata();
         auto braggFitMetadata = dynamic_cast<AtBraggFitMetadata *>(fitTrackMetadata.get());

         // Check for punch-through and if the ELoss was reconstructed in the first place.
         bool isPunchThrough = braggFitMetadata->GetIsPunchThrough();
         bool isReconstructedELoss = braggFitMetadata->GetIsReconstructedELoss();
         if (isPunchThrough)
            nPunchThrough++;

         if (!isReconstructedELoss)
            nNotReconstructedELoss++;

         if (isPunchThrough || !isReconstructedELoss) continue;

         // We are interested in protons, so we check the events that were succesfully identified as protons.
         AtFittedTrack::ParticleInfo particleInfo = fittedTrack->GetParticleInfo();
         TString pdgCode = particleInfo.idPDG;
         int charge = particleInfo.charge;
         double mass = particleInfo.mass;
         //if (pdgCode != "1000010010") continue;
         if (charge != 1) continue;

         // If all checks passed, we can get event information and fill histograms.
         AtFittedTrack::Kinematics kinematics = fittedTrack->GetKinematics();
         double trackKineticEnergy = kinematics.kineticEnergy;
         double trackThetaLAB = kinematics.theta * 180 / TMath::Pi();
         double trackPhi = kinematics.phi * 180 / TMath::Pi();

         auto vertex = fittedTrack->GetVertex();

         double chi2 = braggFitMetadata->GetChi2();

         histChi2->Fill(chi2);

         // Store event IDs depending on different cuts.
         if (cutArtifactKinematics)
            if (cutArtifactKinematics->IsInside(trackThetaLAB, trackKineticEnergy))
               eventsArtifactKinematicsFile << "Event " << i << " | Track " << (trackNum - 1) << "\n";

         if (cutKinematics)
            if (cutKinematics->IsInside(trackThetaLAB, trackKineticEnergy))
               eventsKinematicsFile << "Event " << i << " | Track " << (trackNum - 1) << "\n";


         if(chi2 < 80)
            eventsChi2_1File << "Event " << i << " | Track " << (trackNum - 1) << "\n";
         if(chi2 > 90 && chi2 < 170)
            eventsChi2_2File << "Event " << i << " | Track " << (trackNum - 1) << "\n";
         if(chi2 > 190 && chi2 < 220)
            eventsChi2_3File << "Event " << i << " | Track " << (trackNum - 1) << "\n";

         // Any other gates for the histograms.
         //if(chi2 > 80) continue;
         //if(chi2 < 90 || chi2 > 170) continue;
         //if(chi2 < 190 || chi2 > 220) continue;

         int nZSection = 5;
         if(100 * nZSection > vertex.Z() || vertex.Z() > 100 * (nZSection + 1)) continue;

         histVertexZvTrackThetaLABATTPC->Fill(vertex.Z(), trackThetaLAB);
         histTrackKinematicsATTPC->Fill(trackThetaLAB, trackKineticEnergy);

         // Finally, if there is good Si array measurement, fill the coincidence histograms.
         if (goodSiMeasurement) {
            histVertexZvTrackThetaLABTotalAcceptance->Fill(vertex.Z(), trackThetaLAB);
            histTrackKinematicsTotalAcceptance->Fill(trackThetaLAB, trackKineticEnergy);
         }
      }

   }

   // Close files.
   digiFile->Close();
   eventsArtifactKinematicsFile.close();
   eventsKinematicsFile.close();
   eventsChi2_1File.close();
   eventsChi2_2File.close();
   eventsChi2_3File.close();

   // Draw histograms in TCanvas.
   TCanvas *c = new TCanvas();
   histVertexZvTrackThetaLABATTPC->Draw("zcol");
   histVertexZvTrackThetaLABATTPC->GetXaxis()->SetTitle("Z_{vertex} [mm]");
   histVertexZvTrackThetaLABATTPC->GetYaxis()->SetTitle("#theta_{LAB} [deg]");

   TGraph *kineGSStart = ReadKinematics("./kineFiles/12Be_dp_gs_21MeVu_start.txt");
   TGraph *kineGSEnd = ReadKinematics("./kineFiles/12Be_dp_gs_21MeVu_end.txt");

   TCanvas *c2 = new TCanvas();
   histTrackKinematicsATTPC->Draw("zcol");
   kineGSStart->Draw("same");
   kineGSEnd->Draw("same");
   if (cutArtifactKinematics) cutArtifactKinematics->Draw("same");
   if (cutKinematics) cutKinematics->Draw("same");
   histTrackKinematicsATTPC->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histTrackKinematicsATTPC->GetYaxis()->SetTitle("K_{LAB} [MeV]");

   TCanvas *c3 = new TCanvas();
   histChi2->Draw();
   histChi2->GetXaxis()->SetTitle("#chi^{2}");

   TCanvas *c4 = new TCanvas();
   histVertexZvTrackThetaLABSiArray->Draw("zcol");
   histVertexZvTrackThetaLABSiArray->GetXaxis()->SetTitle("Z_{vertex} [mm]");
   histVertexZvTrackThetaLABSiArray->GetYaxis()->SetTitle("#theta_{LAB} [deg]");

   TCanvas *c5 = new TCanvas();
   histTrackKinematicsSiArray->Draw("zcol");
   kineGSStart->Draw("same");
   kineGSEnd->Draw("same");
   if (cutArtifactKinematics) cutArtifactKinematics->DrawClone("same");
   if (cutKinematics) cutKinematics->DrawClone("same");
   histTrackKinematicsSiArray->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histTrackKinematicsSiArray->GetYaxis()->SetTitle("K_{LAB} [MeV]");

   TCanvas *c6 = new TCanvas();
   histVertexZvTrackThetaLABTotalAcceptance->Draw("zcol");
   histVertexZvTrackThetaLABTotalAcceptance->GetXaxis()->SetTitle("Z_{vertex} [mm]");
   histVertexZvTrackThetaLABTotalAcceptance->GetYaxis()->SetTitle("#theta_{LAB} [deg]");

   TCanvas *c7 = new TCanvas();
   histTrackKinematicsTotalAcceptance->Draw("zcol");
   kineGSStart->Draw("same");
   kineGSEnd->Draw("same");
   if (cutArtifactKinematics) cutArtifactKinematics->DrawClone("same");
   if (cutKinematics) cutKinematics->DrawClone("same");
   histTrackKinematicsTotalAcceptance->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histTrackKinematicsTotalAcceptance->GetYaxis()->SetTitle("K_{LAB} [MeV]");

   TCanvas *c8 = new TCanvas();
   histVertexZvTrackThetaLABTotalSimulation->Draw("zcol");
   histVertexZvTrackThetaLABTotalSimulation->GetXaxis()->SetTitle("Z_{vertex} [mm]");
   histVertexZvTrackThetaLABTotalSimulation->GetYaxis()->SetTitle("#theta_{LAB} [deg]");

   TCanvas *c9 = new TCanvas();
   histTrackKinematicsTotalSimulation->Draw("zcol");
   kineGSStart->Draw("same");
   kineGSEnd->Draw("same");
   if (cutArtifactKinematics) cutArtifactKinematics->DrawClone("same");
   if (cutKinematics) cutKinematics->DrawClone("same");
   histTrackKinematicsTotalSimulation->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histTrackKinematicsTotalSimulation->GetYaxis()->SetTitle("K_{LAB} [MeV]");

   TCanvas *c10 = new TCanvas();
   histHeavyELoss->Draw();
   histHeavyELoss->GetXaxis()->SetTitle("ELoss_{heavy} [idk]");

   // Print stats.
   std::cout << "Percentage of punch-through: " << double(nPunchThrough) * 2 / nDigiEvents << std::endl;
   std::cout << "Percentage of no ELoss reconstructed: " << double(nNotReconstructedELoss) * 2 / nDigiEvents << std::endl;
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
