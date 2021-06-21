#include "genfitSA.h"
#include "AtFitter.h"
#include "AtGenfit.h"

#define cRED "\033[1;31m"
#define cYELLOW "\033[1;33m"
#define cNORMAL "\033[0m"
#define cGREEN "\033[1;32m"

int main()
{
  // -----   Timer   --------------------------------------------------------
  TStopwatch timer;
  timer.Start();
  
  
   //Analysis parameters
   Float_t magneticField = 3.0;//T
   Float_t dMass = 2.0135532;
   Float_t pMass = 1.00727646;
   Int_t atomicNumber = 1;
   Int_t pPDGCode = 2212;
   Int_t dPDGCode = 1000010020;

   const Double_t M_Ener = dMass * 931.49401 / 1000.0;

   // Histograms
   TH1F *angle = new TH1F("angle", "angle", 720, 0, 179);
   TH1F *phi = new TH1F("phi", "phi", 1440, -359, 359);
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

   //Paths
   TString dir = getenv("VMCWORKDIR");
   TString geoManFile  = dir + "/geometry/ATTPC_D1bar_v2_geomanager.root";
   std::cout << " Geometry file : " << geoManFile.Data() << "\n";
   TString filePath ="/macro/Simulation/ATTPC/10Be_dp/";
   TString fileName = "output_digi.root";
   std::vector<TString> files{"output_digi_el_20deg.root"};
   // std::vector<TString> files{"output_digi_el_wide.root","output_digi_inel_wide.root"};
   TString fileNameWithPath = dir + filePath + fileName;

   FairRunAna *run = new FairRunAna();
   run->SetGeomFile(geoManFile.Data());
   TFile *file = new TFile(fileNameWithPath.Data(), "READ");

   // GENFIT geometry
   new TGeoManager("Geometry", "ATTPC geometry");
   TGeoManager::Import(geoManFile.Data());
   genfit::MaterialEffects::getInstance()->init(new genfit::TGeoMaterialInterface());
   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0.,magneticField*10.0)); //

   // event display
   genfit::EventDisplay *display = genfit::EventDisplay::getInstance();

   AtFITTER::AtFitter *fFitter = new AtFITTER::AtGenfit(magneticField, 0.00001, 1000.0);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetPDGCode(dPDGCode);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetMass(dMass);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetAtomicNumber(atomicNumber);
   dynamic_cast<AtFITTER::AtGenfit *>(fFitter)->SetNumFitPoints(1.0);

   for (auto iFile = 0; iFile < files.size(); ++iFile) {

      fileNameWithPath = dir + filePath + files.at(iFile).Data();
      std::cout << " Opening File : " << fileNameWithPath.Data() << std::endl;

      file = new TFile(fileNameWithPath.Data(), "READ");

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = 2; // tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
      // TTreeReaderValue<std::vector<genfit::Track>> fitterVector(Reader1, "ATTPC");

      for (Int_t i = 0; i < nEvents; i++) {

         std::cout << " Event Number : " << i << "\n";

         Reader1.Next();

         AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);

         if (patternEvent) {

            std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
            std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";

            for (auto track : patternTrackCand) {

	      if (track.GetIsNoise() || track.GetHitClusterArray()->size()<5)
                  continue;

               std::cout << " Track " << track.GetTrackID() << " with " << track.GetHitClusterArray()->size()
                         << " clusters "
                         << "\n";
               TVector3 iniPos =
                  track.GetHitClusterArray()->back().GetPosition(); // TODO Check first cluster is the first in time
               std::cout << " Initial position : " << iniPos.X() << " - " << iniPos.Y() << " - " << iniPos.Z() << "\n";

               Double_t dist = TMath::Sqrt(iniPos.X() * iniPos.X() + iniPos.Y() * iniPos.Y());

               if (dist > 100.0)
                  continue;

               fFitter->Init();

               genfit::Track *fitTrack = fFitter->FitTracks(&track);
               TVector3 pos_res;
               TVector3 mom_res;
               TMatrixDSym cov_res;

               try {

                  if (fitTrack && fitTrack->hasKalmanFitStatus()) {

                     auto KalmanFitStatus = fitTrack->getKalmanFitStatus();

                     if (KalmanFitStatus->isFitConverged(false)) {
                        KalmanFitStatus->Print();
                        genfit::MeasuredStateOnPlane fitState = fitTrack->getFittedState();
                        fitState.Print();
                        fitState.getPosMomCov(pos_res, mom_res, cov_res);
                        display->addEvent(fitTrack);
                        angle->Fill(mom_res.Theta() * TMath::RadToDeg());
                        // std::cout<<" Angle "<<mom_res.Theta()<<"\n";
                        auto pos_radial = TMath::Sqrt(TMath::Power(pos_res.X(), 2) + TMath::Power(pos_res.Y(), 2));
                        momentum->Fill(mom_res.Mag());
                        angle_vs_momentum->Fill(mom_res.Theta() * TMath::RadToDeg(), mom_res.Mag());
                        pos_vs_momentum->Fill(pos_res.Mag(), mom_res.Mag());
                        auto len = fitTrack->getTrackLen();
                        length_vs_momentum->Fill(len, mom_res.Mag());
                        auto numHits = fitTrack->getNumPoints();
                        hits_vs_momentum->Fill(numHits, mom_res.Mag());
                        Double_t E = TMath::Sqrt(TMath::Power(mom_res.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener;
                        angle_vs_energy->Fill(mom_res.Theta() * TMath::RadToDeg(), E * 1000.0);
                        phi->Fill(mom_res.Phi() * TMath::RadToDeg());
                     }
                  }
               } catch (std::exception &e) {
                  std::cout << " " << e.what() << "\n";
                  break;
               }

               Double_t theta =
                  180.0 * TMath::DegToRad() - track.GetGeoTheta(); // 180.0*TMath::DegToRad()-track.GetGeoTheta();
               Double_t radius = track.GetGeoRadius() / 1000.0;    // mm to m
               Double_t phi = track.GetGeoPhi();
               Double_t brho = magneticField * radius / TMath::Sin(theta); // Tm
               std::tuple<Double_t, Double_t> mom_ener = GetMomFromBrho(dMass, atomicNumber, brho);
               angle_vs_energy_pattern->Fill(theta * TMath::RadToDeg(), std::get<1>(mom_ener) * 1000.0);
               phi_pattern->Fill(phi * TMath::RadToDeg());
               phi_phi_pattern->Fill(phi * TMath::RadToDeg(), mom_res.Phi() * TMath::RadToDeg());

            } // track loop

         } // if pattern event

      } // Event

   } // File

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
   Kine_AngRec_EnerRec_in->SetLineWidth(1);
   Kine_AngRec_EnerRec_in->SetLineColor(kBlue);
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");
   cKine->cd(2);
   angle_vs_energy_pattern->Draw();
   Kine_AngRec_EnerRec->Draw("SAME");
   Kine_AngRec_EnerRec_in->Draw("ZCOL SAME");

   TCanvas *c2 = new TCanvas();
   c2->Divide(2, 2);
   c2->Draw();
   c2->cd(1);
   length_vs_momentum->Draw();
   c2->cd(2);
   hits_vs_momentum->Draw();

   c2->cd(3);
   phi->Draw();

   c2->cd(4);
   phi_pattern->Draw();

   TCanvas *c3 = new TCanvas();
   c3->Draw();
   phi_phi_pattern->Draw();

   // open event display
   display->open();

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
