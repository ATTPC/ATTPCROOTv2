#include "genfit_Test.hh"
#include "TDatabasePDG.h"

int main()
{

   const Double_t kAu2Gev = 0.9314943228;
   const Double_t khSlash = 1.0545726663e-27;
   const Double_t kErg2Gev = 1 / 1.6021773349e-3;
   const Double_t khShGev = khSlash * kErg2Gev;
   const Double_t kYear2Sec = 3600 * 24 * 365.25;

   TDatabasePDG *db = TDatabasePDG::Instance();
   db->AddParticle("Deuteron", "Deuteron", 2 * kAu2Gev + 8.071e-3, kTRUE, 0, 3, "Ion", 1000010020);
   db->AddParticle("Triton", "Triton", 3 * kAu2Gev + 14.931e-3, kFALSE, khShGev / (12.33 * kYear2Sec), 3, "Ion",
                   1000010030);
   db->AddParticle("Alpha", "Alpha", 4 * kAu2Gev + 2.424e-3, kTRUE, khShGev / (12.33 * kYear2Sec), 6, "Ion",
                   1000020040);
   db->AddParticle("HE3", "HE3", 3 * kAu2Gev + 14.931e-3, kFALSE, 0, 6, "Ion", 1000020030);

   const Double_t M_Ener = 4.00150618 * 931.49401 / 1000.0; // alpha

   FairRunAna *run = new FairRunAna(); // Forcing a dummy run
   TString FileName =
      "/mnt/simulations/attpcroot/fair_install_2020/ATTPCROOTv2_develop/macro/Simulation/ATTPC/16O_aa/output_reco.root";
   std::cout << " Opening File : " << FileName.Data() << std::endl;
   TFile *file = new TFile(FileName.Data(), "READ");

   TTree *tree = (TTree *)file->Get("cbmsim");
   Int_t nEvents = tree->GetEntries();
   std::cout << " Number of events : " << nEvents << std::endl;

   TTreeReader Reader1("cbmsim", file);
   // TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");
   TTreeReaderValue<std::vector<genfit::Track>> fitterVector(Reader1, "ATTPC");

   // GENFIT geometry
   new TGeoManager("Geometry", "ATTPC geometry");
   TGeoManager::Import(
      "/mnt/simulations/attpcroot/fair_install_2020/ATTPCROOTv2_develop/geometry/ATTPC_He1bar_v2_geomanager.root");
   genfit::MaterialEffects::getInstance()->init(new genfit::TGeoMaterialInterface());
   genfit::FieldManager::getInstance()->init(new genfit::ConstField(0., 0., 30.)); //

   // Histograms
   TH1F *angle = new TH1F("angle", "angle", 720, 0, 179);
   TH1F *momentum = new TH1F("momentum", "momentum", 1000, 0, 2.0); // GeV
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);
   TH2F *pos_vs_momentum = new TH2F("pos_vs_momentum", "pos_vs_momentum", 200, 0, 200, 1000, 0, 2.0);
   TH2F *length_vs_momentum = new TH2F("length_vs_momentum", "length_vs_momentum", 200, 0, 200, 1000, 0, 2.0);
   TH2F *hits_vs_momentum = new TH2F("hits_vs_momentum", "hits_vs_momentum", 200, 0, 200, 1000, 0, 2.0);
   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 1000, 0, 100.0);

   // event display
   genfit::EventDisplay *display = genfit::EventDisplay::getInstance();

   for (Int_t i = 0; i < nEvents; i++) {

      std::cout << " Event Number : " << i << "\n";

      Reader1.Next();
      // eventArray->Clear();
      // AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);

      auto numGenfitTracks = fitterVector->size();
      std::cout << " Number of tracks : " << numGenfitTracks << std::endl;

      for (auto iTrack = 0; iTrack < numGenfitTracks; ++iTrack) {

         TVector3 pos_res;
         TVector3 mom_res;
         TMatrixDSym cov_res;

         try {
            auto genfitTrack = fitterVector->at(iTrack);
            // genfitTrack.Print();
            if (genfitTrack.hasKalmanFitStatus()) {
               auto KalmanFitStatus = genfitTrack.getKalmanFitStatus();

               if (KalmanFitStatus->isFitted()) {
                  KalmanFitStatus->Print();
                  genfit::MeasuredStateOnPlane fitState = genfitTrack.getFittedState();
                  fitState.Print();
                  fitState.getPosMomCov(pos_res, mom_res, cov_res);
                  display->addEvent(&genfitTrack);
                  angle->Fill(mom_res.Theta() * TMath::RadToDeg());
                  // std::cout<<" Angle "<<mom_res.Theta()<<"\n";
                  auto pos_radial = TMath::Sqrt(TMath::Power(pos_res.X(), 2) + TMath::Power(pos_res.Y(), 2));
                  momentum->Fill(mom_res.Mag());
                  angle_vs_momentum->Fill(mom_res.Theta() * TMath::RadToDeg(), mom_res.Mag());
                  pos_vs_momentum->Fill(pos_res.Mag(), mom_res.Mag());
                  auto len = genfitTrack.getTrackLen();
                  length_vs_momentum->Fill(len, mom_res.Mag());
                  auto numHits = genfitTrack.getNumPoints();
                  hits_vs_momentum->Fill(numHits, mom_res.Mag());
                  Double_t E = TMath::Sqrt(TMath::Power(mom_res.Mag(), 2) + TMath::Power(M_Ener, 2)) - M_Ener;
                  angle_vs_energy->Fill(mom_res.Theta() * TMath::RadToDeg(), E * 1000.0);
               }
            }

         } catch (std::exception &e) {
            std::cout << " " << e.what() << "\n";
            break;
         }
      }

      /*if (patternEvent) {
         std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
         std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";
         for (auto track : patternTrackCand) {
            std::vector<AtHit> *hitArray = track.GetHitArray();
            for (auto hit : *hitArray) {
               TVector3 pos = hit.GetPosition();
               int TB = hit.GetTimeStamp();
               std::cout << " Pos : " << pos.X() << "	" << pos.Y() << "	" << pos.Z() << " " << TB << "\n";
            }
         }
    }*/
   }

   Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "../O16_aa_el_kine.txt";
   std::ifstream *kineStr = new std::ifstream(fileKine.Data());
   Int_t numKin = 0;

   if (!kineStr->fail()) {
      while (!kineStr->eof()) {
         *kineStr >> ThetaCMS[numKin] >> ThetaLabRec[numKin] >> EnerLabRec[numKin] >> ThetaLabSca[numKin] >>
            EnerLabSca[numKin];
         // numKin++;

         // MomLabRec[numKin] =( pow(EnerLabRec[numKin] + M_Ener,2) - TMath::Power(M_Ener, 2))/1000.0;
         // std::cout<<" Momentum : " <<MomLabRec[numKin]<<"\n";
         // Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
         numKin++;
      }
   } else if (kineStr->fail())
      std::cout << " Warning : No Kinematics file found for this reaction!" << std::endl;

   TGraph *Kine_AngRec_EnerRec = new TGraph(numKin, ThetaLabRec, EnerLabRec);

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
   cKine->Draw();
   angle_vs_energy->Draw();
   Kine_AngRec_EnerRec->SetLineColor(kRed);
   Kine_AngRec_EnerRec->Draw("SAME");

   TCanvas *c2 = new TCanvas();
   c2->Divide(2, 2);
   c2->Draw();
   c2->cd(1);
   length_vs_momentum->Draw();
   c2->cd(2);
   hits_vs_momentum->Draw();

   // open event display
   display->open();
}
