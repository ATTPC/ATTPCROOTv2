void GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E);

void O16_ana()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run
   TString FileName = "output_digi.root";
   std::cout << " Opening File : " << FileName.Data() << std::endl;
   TFile *file = new TFile(FileName.Data(), "READ");


   TTree *tree = (TTree *)file->Get("cbmsim");
   Int_t nEvents = tree->GetEntries();
   std::cout << " Number of events : " << nEvents << std::endl;

   TTreeReader Reader1("cbmsim", file);
   TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtPatternEvent");

   TH2F *angle_vs_energy = new TH2F("angle_vs_energy", "angle_vs_energy", 720, 0, 179, 1000, 0, 100.0);
   TH2F *angle_vs_momentum = new TH2F("angle_vs_momentum", "angle_vs_momentum", 720, 0, 179, 1000, 0, 2.0);
   
   for (Int_t i = 0; i < nEvents; i++) {

     //eventArray->Clear();
     
      std::cout << " Event Number : " << i << "\n";

      Reader1.Next();
      
      AtPatternEvent *patternEvent = (AtPatternEvent *)eventArray->At(0);

      if (patternEvent) {
         std::vector<AtTrack> &patternTrackCand = patternEvent->GetTrackCand();
         std::cout << " Number of pattern tracks " << patternTrackCand.size() << "\n";
         for (auto track : patternTrackCand) {

	   Double_t theta = track.GetGeoTheta();
	   Double_t rad   = track.GetGeoRadius();
      Double_t phi = track.GetGeoPhi();

      Double_t B_f = 3.0;
	   
	   double bro = B_f*rad/TMath::Sin(theta)/1000.0;
           double ener = 0;
	   Double_t  Am = 4.0;

           GetEnergy(Am,2.0,bro,ener);

	   angle_vs_energy->Fill(theta*TMath::RadToDeg(),ener*Am);

      std::cout << " Brho : " << bro << " - Theta : " << theta * TMath::RadToDeg()
                << " - Phi : " << phi * TMath::RadToDeg() << " - Radius : " << rad << " - Energy :" << ener * Am
                << "\n";

      /*std::vector<AtHit> *hitArray = track.GetHitArray();
            for (auto hit : *hitArray) {
               TVector3 pos = hit.GetPosition();
               int TB = hit.GetTimeStamp();
               std::cout << " Pos : " << pos.X() << "   " << pos.Y() << "       " << pos.Z() << " " << TB << "\n";
	       }*/
	}
    }


      
   }//nEvents

    Double_t *ThetaCMS = new Double_t[20000];
   Double_t *ThetaLabRec = new Double_t[20000];
   Double_t *EnerLabRec = new Double_t[20000];
   Double_t *ThetaLabSca = new Double_t[20000];
   Double_t *EnerLabSca = new Double_t[20000];
   Double_t *MomLabRec = new Double_t[20000];

   TString fileKine = "O16_aa_el_kine.txt";
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
   c1->Draw();
   angle_vs_energy->Draw();
   Kine_AngRec_EnerRec->SetLineColor(kRed);
   Kine_AngRec_EnerRec->Draw("SAME");

   
   
}

void GetEnergy(Double_t M,Double_t IZ,Double_t BRO,Double_t &E){

  //Energy per nucleon
  Float_t  AM=931.5;
  Float_t X=BRO/0.1439*IZ/M;
  X=pow(X,2);
  X=2.*AM*X;
  X=X+pow(AM,2);
  E=TMath::Sqrt(X)-AM;

}




