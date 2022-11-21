void MCanalysis()
{

   TH2D *ElossRangeBeam = new TH2D("ElossRangeBeam", "ElossRangeBeam", 600, -300, 300, 200, 0, 200);
   ElossRangeBeam->SetMarkerStyle(20);
   ElossRangeBeam->SetMarkerSize(0.5);
   TH2D *ElossRangeScatter = new TH2D("ElossRangeScatter", "ElossRangeScatter", 1000, 0, 300, 1000, 0, 20);
   ElossRangeScatter->SetMarkerStyle(20);
   ElossRangeScatter->SetMarkerSize(0.5);
   TH2D *ElossAngleScatter = new TH2D("ElossAngleScatter", "ElossAngleScatter", 1000, 0, 180, 200, 0, 20);
   ElossAngleScatter->SetMarkerStyle(20);
   ElossAngleScatter->SetMarkerSize(0.5);
   TH2D *KineScatter = new TH2D("KineScatter", "KineScatter", 1000, 0, 180, 1000, 0, 40);
   KineScatter->SetMarkerStyle(20);
   KineScatter->SetMarkerSize(1.0);
   TH2D *ScatterEvsEloss = new TH2D("ScatterEvsEloss", "ScatterEvsEloss", 1000, 0, 10, 1000, 0, 10);

   // AtTpcPoint* point = new AtTpcPoint();
   TClonesArray *pointArray = 0;

   TFile *file = new TFile("./data/attpcsim.root", "READ");
   TTree *tree = (TTree *)file->Get("cbmsim");

   tree = (TTree *)file->Get("cbmsim");
   tree->SetBranchAddress("AtTpcPoint", &pointArray);
   Int_t nEvents = tree->GetEntriesFast();

   std::cout << " Number of events " << nEvents << "\n";

   for (Int_t iEvent = 0; iEvent < nEvents; iEvent++) {
      TString VolName;
      tree->GetEvent(iEvent);
      Int_t n = pointArray->GetEntries();
      std::cout << " Event Number : " << iEvent << " with " << n << " points." << std::endl;

      Double_t beamVertex = 0.0;
      Double_t beamEloss = 0.0;
      Double_t scatterRange = 0.0;
      Double_t scatterEloss = 0.0;
      Double_t scatterAngle = 0.0;
      Double_t scatterEnergy = 0.0;

      for (Int_t i = 0; i < n; i++) {
         auto point = (AtMCPoint *)pointArray->At(i);
         auto VolName = point->GetVolName();
         auto trackID = point->GetTrackID();
         auto angle = point->GetAIni();
         auto energy = point->GetEIni();
         auto z = point->GetAtomicNum();

         if (VolName == "target_drift_volume" && z == 12) { // 22Mg in the beam pipe

            beamEloss += (point->GetEnergyLoss()) * 1000; // MeV
            beamVertex = point->GetZ() * 10;              // mm

            // if(beamVertex>60) std::cout<<" Volume Name : "<<VolName<<" - Track ID : "<<trackID<<" Angle : "<<angle<<"
            // - Atomic Number :"<<z<<" Vertex "<<beamVertex<<" Energy Loss "<<beamEloss<<std::endl;

         } else if (VolName == "drift_volume" && (z == 2 || z == 1)) {

            scatterEloss += (point->GetEnergyLoss()) * 1000; // MeV
            scatterRange = point->GetLength() * 10;          // mm
            scatterEnergy = point->GetEIni();
            scatterAngle = point->GetAIni();

            // std::cout << " Volume Name : " << VolName << " - Track ID : " << trackID << " - Energy : " << energy << "
            // - Angle : " << angle << " - Atomic Number : " << z << std::endl;

         } // Volume selection
      }    // Point loop

      // Filling histograms
      std::cout << " ---------- Beam energy loss " << beamEloss << " at Z vertex : " << beamVertex << "\n";
      ElossRangeBeam->Fill(beamVertex, beamEloss);
      ElossRangeScatter->Fill(scatterRange, scatterEloss);
      ElossAngleScatter->Fill(scatterAngle, scatterEloss);
      KineScatter->Fill(scatterAngle, scatterEnergy);
      ScatterEvsEloss->Fill(scatterEnergy, scatterEloss);

   } // Event loop

   // Plots
   TCanvas *c1 = new TCanvas();
   c1->Divide(2, 2);
   c1->Draw();
   c1->cd(1);
   ElossRangeBeam->Draw("zcol");
   ElossRangeBeam->GetYaxis()->SetTitle("Energy Loss (Mev)");
   ElossRangeBeam->GetXaxis()->SetTitle("22Mg reaction Vertex (mm)");
   c1->cd(2);
   ElossRangeScatter->Draw("histo");
   ElossRangeScatter->GetYaxis()->SetTitle("Energy Loss (Mev)");
   ElossRangeScatter->GetXaxis()->SetTitle("Scattered range in Ar+N2 (80-20) (mm)");
   c1->cd(3);
   KineScatter->Draw("histo");
   KineScatter->GetYaxis()->SetTitle("Scattered energy  (Mev)");
   KineScatter->GetXaxis()->SetTitle("Scattered angle (deg)");
   c1->cd(4);
   ElossAngleScatter->Draw("histo");
   ElossAngleScatter->GetYaxis()->SetTitle("Scattered energy  (Mev)");
   ElossAngleScatter->GetXaxis()->SetTitle("Scattered angle (deg)");

   TCanvas *c2 = new TCanvas();
   c2->Divide(2, 2);
   c2->Draw();
   c2->cd(1);
   ScatterEvsEloss->Draw("zcol");
}
