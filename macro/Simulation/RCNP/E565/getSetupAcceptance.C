
TGraph* ReadKinematics(TString kineFile);

void getSetupAcceptance()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   // Histogram definitions.
   TH2F *histVertexZvTrackThetaLAB = new TH2F("histVertexZvTrackThetaLAB", "histVertexZvTrackThetaLAB", 100, 0, 1000, 180, 0, 180);
   TH2F *histTrackKinematics = new TH2F("histTrackKinematics", "histTrackKinematics", 180, 0, 180, 80, 0, 20);



   TH1F *histChi2 = new TH1F("histChi2", "histChi2", 100, 0, 1000);

   // Min and Max angles of the simulation.
   Double_t ThetaMinCMS = 0.0;
   Double_t ThetaMaxCMS = 50.0;

   // Open the digitalization file and get the TTree.
   TString digiFileName = TString::Format("./output_digi_rcnp_13Be_p_%.1f_%.1f_600Torr_9mmBinning.root", ThetaMinCMS, ThetaMaxCMS);
   TFile *file = new TFile(digiFileName, "READ");
   TTree *tree = (TTree *)file->Get("cbmsim");
   Int_t nEvents = tree->GetEntries();
   std::cout << " Number of simulated events : " << double(nEvents) / 2 << std::endl;

   // Creare the TTreeReader to read the AtTrackingEvents.
   TTreeReader reader("cbmsim", file);
   TTreeReaderValue<TClonesArray> trackingArray(reader, "AtTrackingEvent");

   // Number of fails.
   int nPunchThrough{};
   int nNotReconstructedELoss{};

   // Loop over events.
   for (int i = 0; i < nEvents; i++) {
      reader.Next();
      AtTrackingEvent *trackingEvent = (AtTrackingEvent *)trackingArray->At(0);

      if (!trackingEvent)
         continue;

      auto &fittedTracks = trackingEvent->GetFittedTracks();

      for (auto &fittedTrack: fittedTracks) {

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

         if (isPunchThrough || !isReconstructedELoss)
            continue;

         // We are interested in protons, so we check the events that were succesfully identified as protons.
         AtFittedTrack::ParticleInfo particleInfo = fittedTrack->GetParticleInfo();
         TString pdgCode = particleInfo.idPDG;
         int charge = particleInfo.charge;
         double mass = particleInfo.mass;
         if (pdgCode != "1000010010")
            continue;

         // If all checks passed, we can get event information and fill histograms.
         AtFittedTrack::Kinematics kinematics = fittedTrack->GetKinematics();
         double trackKineticEnergy = kinematics.kineticEnergy;
         double trackThetaLAB = 180 - kinematics.theta * 180 / TMath::Pi();
         double trackPhi = kinematics.phi * 180 / TMath::Pi();

         auto vertex = fittedTrack->GetVertex();

         double chi2 = braggFitMetadata->GetChi2();

         histChi2->Fill(chi2);

         //if(chi2 > 80)
         //if(chi2 < 90 || chi2 > 170)
         //if(chi2 < 190 || chi2 > 220)
            //continue;

         histVertexZvTrackThetaLAB->Fill(vertex.Z(), trackThetaLAB);
         histTrackKinematics->Fill(trackThetaLAB, trackKineticEnergy);
      }

   }

   // Close file.
   file->Close();

   // Draw histograms in TCanvas.
   TCanvas *c = new TCanvas();
   histVertexZvTrackThetaLAB->Draw("zcol");
   histVertexZvTrackThetaLAB->GetXaxis()->SetTitle("Z_{vertex} [mm]");
   histVertexZvTrackThetaLAB->GetYaxis()->SetTitle("#theta_{LAB} [deg]");

   TGraph *kineGS = ReadKinematics("./12Be_dp_gs_21MeVu.txt");

   TCanvas *c2 = new TCanvas();
   histTrackKinematics->Draw("zcol");
   kineGS->Draw("same");
   histTrackKinematics->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histTrackKinematics->GetYaxis()->SetTitle("K_{LAB} [MeV]");

   TCanvas *c3 = new TCanvas();
   histChi2->Draw();
   histChi2->GetXaxis()->SetTitle("#chi^{2}");

   // Print stats.
   std::cout << "Percentage of punch-through: " << double(nPunchThrough) * 2 / nEvents << std::endl;
   std::cout << "Percentage of no ELoss reconstructed: " << double(nNotReconstructedELoss) * 2 / nEvents << std::endl;
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
