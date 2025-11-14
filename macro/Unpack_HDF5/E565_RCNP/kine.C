TGraph* ReadKinematics(TString kineFile);
Double_t omega(Double_t x, Double_t y, Double_t z);
std::tuple<double, double> kine_2b(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject);

void kine()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   // Relevant masses, beam energy, etc...
   const double u_to_MeV = 931.49401;

   const double m_12Be = 12.02473  * u_to_MeV;
   const double m_d    = 2.0135532 * u_to_MeV;
   const double m_13Be = 13.03394  * u_to_MeV;
   const double m_p    = 1.00783   * u_to_MeV;

   const double EBeam = 21.0 * 12.02473; // MeV

   // AtMap to check if a hit belong to a big pad or small pad.
   TString scriptfile = "rcnp_map_size.xml";
   TString dir = getenv("VMCWORKDIR");
   TString mapDir = dir + "/scripts/" + scriptfile;
   AtTpcMap *map = new AtTpcMap();
   map->ParseXMLMap(mapDir.Data());
   map->GeneratePadPlane();

   // Vertex finder.
   double lineDistThreshold = 30;
   int numTracksPerVtx = 1;
   AtFindVertex findVtx(lineDistThreshold);

   // ELoss model for kinetic energy estimations.
   double density = 1.4232e-3; // 500Torr
   std::vector<std::tuple<int, int, int>> materialComponents;
   materialComponents.push_back(std::make_tuple(12, 6, 3));
   materialComponents.push_back(std::make_tuple(2, 1, 8));

   std::unique_ptr<AtTools::AtELossCATIMA> eLossModelC3D8_d = std::make_unique<AtTools::AtELossCATIMA>(density, "CATima_C3D8_500Torr_d");
   eLossModelC3D8_d->SetMaterial(materialComponents);
   eLossModelC3D8_d->SetProjectile(2, 1, 2.0135532);
   eLossModelC3D8_d->SetPDGCode("1000020010");

   // Cut files.
   TFile *cutKineFile = new TFile("./cutFiles/cutKine.root", "READ");
   TCutG *cutKineBProton = (TCutG *)cutKineFile->Get("cutKineBProton");
   TCutG *cutKineAElastic = (TCutG *)cutKineFile->Get("cutKineAElastic");
   cutKineFile->Close();

   TFile *cutPIDFile = new TFile("./cutFiles/PID.root", "READ");
   TCutG *cutPIDA = (TCutG *)cutPIDFile->Get("cutPIDA");
   TCutG *cutPIDB = (TCutG *)cutPIDFile->Get("cutPIDB");
   TCutG *cutPIDC = (TCutG *)cutPIDFile->Get("cutPIDC");
   TCutG *cutPIDD = (TCutG *)cutPIDFile->Get("cutPIDD");
   TCutG *cutPIDE = (TCutG *)cutPIDFile->Get("cutPIDE");
   cutPIDFile->Close();

   // Histogram definitions.
   TH2F *histRangeVThetaLAB = new TH2F("histRangeVThetaLAB", "histRangeVThetaLAB", 180, 0, 180, 1030, 0, 1030);
   TH2F *histEstimatedKinEVThetaLABTotal = new TH2F("histEstimatedKinEVThetaLABTotal", "histEstimatedKinEVThetaLABTotal", 180, 0, 180, 500, 0, 20);
   TH2F *histEstimatedKinEVThetaLABA = new TH2F("histEstimatedKinEVThetaLABA", "histEstimatedKinEVThetaLABA", 180, 0, 180, 500, 0, 20);
   TH2F *histEstimatedKinEVThetaLABB = new TH2F("histEstimatedKinEVThetaLABB", "histEstimatedKinEVThetaLABB", 90, 0, 180, 200, 0, 20);
   TH2F *histEstimatedKinEVThetaLABC = new TH2F("histEstimatedKinEVThetaLABC", "histEstimatedKinEVThetaLABC", 180, 0, 180, 500, 0, 20);
   TH2F *histEstimatedKinEVThetaLABD = new TH2F("histEstimatedKinEVThetaLABD", "histEstimatedKinEVThetaLABD", 180, 0, 180, 500, 0, 20);
   TH2F *histEstimatedKinEVThetaLABE = new TH2F("histEstimatedKinEVThetaLABE", "histEstimatedKinEVThetaLABE", 180, 0, 180, 500, 0, 20);
   TH2F *histdEdxVRange = new TH2F("histdEdxVRange", "histdEdxVRange", 515, 0, 1030, 1600, 0, 8000);

   TH1F *histBraggChi2 = new TH1F("histBraggChi2", "histBraggChi2", 1000, 0, 10000);
   TH2F *histBraggKinematicsATTPC = new TH2F("histBraggKinematicsATTPC", "histBraggKinematicsATTPC", 360, 0, 180, 200, 0, 20);

   std::vector runNums = {1001, 1002, 1003, 1004, 1005, 1006,
                          1008, 1009, 1010,
                          1016, 1017, 1018,
                          1024, 1025, 1026, 1027, 1028, 1029, 1030, 1031, 1032, 1033,
                          1035, 1036, 1037, 1038, 1039, 1040,
                          1042, 1043, 1044,     1046,      1050, 1051,
                          1056, 1057, 1058, 1059, 1060};
   //std::vector runNums = {1050, 1051, 1056, 1057, 1058, 1059, 1060};

   int numKineBProton{};
   int numKineAElastic{};

   for (int runNum: runNums) {
      // Open the digitalization file and get the TTree.
      //TString unpackFileName = TString::Format("/data/ATTPCROOTv2_results/E565/UnpackerOutput/here/run_%04d.root", runNum);
      TString unpackFileName = TString::Format("/data/ATTPCROOTv2_results/E565/UnpackerOutput/reUnpack/run_%04d.root", runNum);
      TFile *unpackFile = new TFile(unpackFileName, "READ");
      TTree *unpackTree = (TTree *)unpackFile->Get("cbmsim");
      int nUnpackEvents = unpackTree->GetEntries();
      std::cout << " Number of unpacked events in run " << runNum << ": " << nUnpackEvents << std::endl;

      // Creare the TTreeReader to read the AtTrackingEvents and simulation.
      TTreeReader unpackReader("cbmsim", unpackFile);
      TTreeReaderValue<TClonesArray> patternArray(unpackReader, "AtPatternEvent");
      //TTreeReaderValue<TClonesArray> trackingArray(unpackReader, "AtTrackingEvent");

      // Loop over events.
      for (int i = 0; i < nUnpackEvents; i++) {
         unpackReader.Next();

         // First, we obtain some rough kinematics just by using the AtPatternEvent.
         AtPatternEvent *patternEvent = (AtPatternEvent *)patternArray->At(0);
         if (!patternEvent) continue;

         // We want to focus on events with 2 or less tracks for now.
         auto &tracks = patternEvent->GetTrackCand();
         //if (tracks.size() > 4) continue;
         if (tracks.size() > 2) continue;
         //if (tracks.size() > 1) continue;

         //findVtx.FindVertex(tracks, numTracksPerVtx);
         //std::vector<tracksFromVertex> tv = findVtx.GetTracksVertex();

         // Iterate over AtTracks and extract their kinematics.
         for (auto &track: tracks) {

           /* bool foundVertex = false;
            XYZPoint vertex;
            for (auto trackVertex : tv) {
               for (auto trackVtx : trackVertex.tracks) {
                  if (trackVtx.GetTrackID() == track.GetTrackID()) {
                     vertex = trackVertex.vertex;
                     foundVertex = true;
                     break;
                  }
               }
               if (foundVertex)
                  break;
            }

            if (!foundVertex)
               continue;*/

            auto *pattern = track.GetPattern();

            auto firstPoint = track.GetFirstPoint();
            auto lastPoint = track.GetLastPoint();
            double roughRangeEstimation = pattern->DistanceAlongPattern(lastPoint, firstPoint);
            double trackThetaLAB = track.GetGeoTheta() * 180 / TMath::Pi();
            double trackPhi = track.GetGeoPhi() * 180 / TMath::Pi();

            //if (trackThetaLAB < 100) continue;

            double estimatedKinE{0.1};
            while (eLossModelC3D8_d->GetRange(estimatedKinE) < roughRangeEstimation)
               estimatedKinE += 0.01;

            double totalCharge{};
            auto braggCurvePairs = track.GetBraggCurveValues();
            auto &hits = track.GetHitArray();
            double rangeInSmallPads{};
	         for (auto &hit: hits) {
               int padNum = hit->GetPadNum();
               int sizeID = map->GetPadSize(padNum);
               if (sizeID == 1)
                  continue;
               totalCharge += hit->GetCharge();
               double currentRangeInSmallPads = pattern->DistanceAlongPattern(hit->GetPosition(), firstPoint);
               if (currentRangeInSmallPads > rangeInSmallPads)
                  rangeInSmallPads = currentRangeInSmallPads;
            }
            double dEdx = totalCharge / rangeInSmallPads;

            //if (!cutElastic->IsInside(trackThetaLAB, roughRangeEstimation)) continue;
            //if (cutPseudoProton->IsInside(roughRangeEstimation, dEdx)) continue;
            //if (!cutPD->IsInside(roughRangeEstimation, dEdx)) continue;
            //if (totalCharge > 50000) continue;

            histRangeVThetaLAB->Fill(trackThetaLAB, roughRangeEstimation);
            histdEdxVRange->Fill(roughRangeEstimation, dEdx);
            histEstimatedKinEVThetaLABTotal->Fill(trackThetaLAB, estimatedKinE);

            if (cutPIDA->IsInside(roughRangeEstimation, dEdx)) {
               histEstimatedKinEVThetaLABA->Fill(trackThetaLAB, estimatedKinE);
               if (cutKineAElastic->IsInside(trackThetaLAB, estimatedKinE))
                  numKineAElastic++;
            }

            if (cutPIDB->IsInside(roughRangeEstimation, dEdx)) {
               histEstimatedKinEVThetaLABB->Fill(trackThetaLAB, estimatedKinE);
               if (cutKineBProton->IsInside(trackThetaLAB, estimatedKinE))
                  numKineBProton++;
            }

            if (cutPIDC->IsInside(roughRangeEstimation, dEdx))
               histEstimatedKinEVThetaLABC->Fill(trackThetaLAB, estimatedKinE);

            if (cutPIDD->IsInside(roughRangeEstimation, dEdx))
               histEstimatedKinEVThetaLABD->Fill(trackThetaLAB, estimatedKinE);

            if (cutPIDE->IsInside(roughRangeEstimation, dEdx))
               histEstimatedKinEVThetaLABE->Fill(trackThetaLAB, estimatedKinE);

         }
/*
         // Now, we try to check how the Bragg curve results look like.
         AtTrackingEvent *trackingEvent = (AtTrackingEvent *)trackingArray->At(0);
         if (!trackingEvent) continue;

         auto &fittedTracks = trackingEvent->GetFittedTracks();
         for (auto &fittedTrack: fittedTracks) {

            // Extract the metadata for this fit.
            auto &fitTrackMetadata = fittedTrack->GetTrackMetadata();
            auto braggFitMetadata = dynamic_cast<AtBraggFitMetadata *>(fitTrackMetadata.get());

            // Check for punch-through and if the ELoss was reconstructed in the first place.
            bool isPunchThrough = braggFitMetadata->GetIsPunchThrough();
            bool isReconstructedELoss = braggFitMetadata->GetIsReconstructedELoss();
            if (isPunchThrough || !isReconstructedELoss) continue;

            // We are interested in protons and deuterons. We check if the charge is 1.
            AtFittedTrack::ParticleInfo particleInfo = fittedTrack->GetParticleInfo();
            int charge = particleInfo.charge;
            if (charge != 1) continue;

            // If all checks passed, we can get event information and fill histograms.
            AtFittedTrack::Kinematics kinematics = fittedTrack->GetKinematics();
            double trackKineticEnergy = kinematics.kineticEnergy;
            double trackThetaLAB = kinematics.theta * 180 / TMath::Pi();
            double trackPhi = kinematics.phi * 180 / TMath::Pi();

            auto vertex = fittedTrack->GetVertex();

            double chi2 = braggFitMetadata->GetChi2();


            int nZSection = 5;
            if(100 * nZSection > vertex.Z() || vertex.Z() > 100 * (nZSection + 1)) continue;
            if (chi2 > 2500) continue;

            histBraggChi2->Fill(chi2);
            histBraggKinematicsATTPC->Fill(trackThetaLAB, trackKineticEnergy);
         }
*/
      }
      // Close files.
      unpackFile->Close();
   }

   // Kinematic lines.
   TGraph *kine_dd = ReadKinematics("./12Be_dd_gs.txt");
   TGraph *kine_iso_dp = ReadKinematics("./12Beiso_dp_gs.txt");
   TGraph *kine_dp = ReadKinematics("./12Be_dp_gs.txt");
   TGraph *kine_12C12C = ReadKinematics("./12Be_12C12C_gs.txt");

   // Draw histograms in TCanvas.
   TCanvas *c = new TCanvas();
   histRangeVThetaLAB->Draw("zcol");
   histRangeVThetaLAB->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histRangeVThetaLAB->GetYaxis()->SetTitle("roughRange [mm]");

   TCanvas *c2 = new TCanvas();
   histEstimatedKinEVThetaLABTotal->Draw("zcol");
   kine_dd->Draw("same");
   kine_dp->Draw("same");
   kine_iso_dp->Draw("same");
   kine_12C12C->Draw("same");
   histEstimatedKinEVThetaLABTotal->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLABTotal->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c3 = new TCanvas();
   histEstimatedKinEVThetaLABA->Draw("zcol");
   kine_dd->Draw("same");
   kine_dp->Draw("same");
   kine_iso_dp->Draw("same");
   kine_12C12C->Draw("same");
   cutKineAElastic->Draw("same");
   histEstimatedKinEVThetaLABA->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLABA->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c4 = new TCanvas();
   histEstimatedKinEVThetaLABB->Draw("zcol");
   kine_dd->Draw("same");
   kine_dp->Draw("same");
   kine_iso_dp->Draw("same");
   kine_12C12C->Draw("same");
   cutKineBProton->Draw("same");
   histEstimatedKinEVThetaLABB->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLABB->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c5 = new TCanvas();
   histEstimatedKinEVThetaLABC->Draw("zcol");
   kine_dd->Draw("same");
   kine_dp->Draw("same");
   kine_iso_dp->Draw("same");
   kine_12C12C->Draw("same");
   histEstimatedKinEVThetaLABC->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLABC->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c6 = new TCanvas();
   histEstimatedKinEVThetaLABD->Draw("zcol");
   kine_dd->Draw("same");
   kine_dp->Draw("same");
   kine_iso_dp->Draw("same");
   kine_12C12C->Draw("same");
   histEstimatedKinEVThetaLABD->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLABD->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c7 = new TCanvas();
   histEstimatedKinEVThetaLABE->Draw("zcol");
   kine_dd->Draw("same");
   kine_dp->Draw("same");
   kine_iso_dp->Draw("same");
   kine_12C12C->Draw("same");
   histEstimatedKinEVThetaLABE->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histEstimatedKinEVThetaLABE->GetYaxis()->SetTitle("roughKinE [MeV]");

   TCanvas *c8 = new TCanvas();
   histdEdxVRange->Draw("zcol");
   cutPIDA->Draw("same");
   cutPIDB->Draw("same");
   cutPIDC->Draw("same");
   cutPIDD->Draw("same");
   cutPIDE->Draw("same");
   histdEdxVRange->GetXaxis()->SetTitle("roughRange [mm]");
   histdEdxVRange->GetYaxis()->SetTitle("#frac{dE}{dx} [ADC/mm]");

   TCanvas *c9 = new TCanvas();
   histBraggKinematicsATTPC->Draw("zcol");
   kine_dd->Draw("same");
   kine_dp->Draw("same");
   histBraggKinematicsATTPC->GetXaxis()->SetTitle("#theta_{LAB} [deg]");
   histBraggKinematicsATTPC->GetYaxis()->SetTitle("BraggKinE [MeV]");

   TCanvas *c10 = new TCanvas();
   histBraggChi2->Draw();
   histBraggChi2->GetXaxis()->SetTitle("#chi^{2}");

   std::cout << "Number of events in elastic region of kinematics A is " << numKineAElastic << "." <<std::endl;
   std::cout << "Number of events in proton region of kinematics B is " << numKineBProton << "." <<std::endl;
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
