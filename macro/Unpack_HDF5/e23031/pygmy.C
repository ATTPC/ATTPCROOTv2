
TGraph* ReadKinematics(TString kineFile);

void pygmy(TString fileName9Li, TString cutsFileName)
{
   // Define some variables and open the file with the cuts.
   Float_t minDistanceFromBorder = 35.0;

   TFile *cutsFile = new TFile(cutsFileName, "READ");
   TCutG *cut9Li = (TCutG *)cutsFile->Get("cut9Li");
   TCutG *cut11Li = (TCutG *)cutsFile->Get("cut11Li");
   cutsFile->Close();

   // Define the histograms to be filled.
   TH2F *hs800PID = new TH2F("hs800PID", "hs800PID", 200, -1200, -200, 200, 0, 1000);

   TH2F *hKinematicsElastic = new TH2F("hKinematicsElastic", "hKinematicsElastic", 180, 0, 180, 200, 0, 4);
   TH1F *hExElastic = new TH1F("hExElastic", "hExElastic", 100, -30, 30);
   TH1F *hAngDistrElastic = new TH1F("hAngDistrElastic", "hAngDistrElastic", 181, -0.5, 180.5);

   TH2F *hKinematics9Li = new TH2F("hKinematics9Li", "hKinematics9Li", 180, 0, 180, 200, 0, 4);
   TH1F *hEx9Li = new TH1F("hEx9Li", "hEx9Li", 100, -30, 30);
   TH1F *hAngDistr9Li = new TH1F("hAngDistr9Li", "hAngDistr9Li", 181, -0.5, 180.5);

   // Open the file with the results from all the 9Li runs.
   TFile *file9Li = new TFile(fileName9Li, "READ");
   TTree *tree9Li = (TTree *)file9Li->Get("anatree");
   Int_t totalEntries9Li = tree9Li->GetEntries();
   std::cout << " Total number of entries with 9Li setting: " << totalEntries9Li << std::endl;

   // Define the variables and set the branch addresses.
   Float_t Ex4{};
   Float_t thetaLAB{};
   Float_t kEnergy{};
   Float_t MaxR{};

   Double_t s800dE{};
   Double_t s800ToF{};
   Double_t s800ToFXf{};

   tree9Li->SetBranchAddress("Ex4", &Ex4);
   tree9Li->SetBranchAddress("theta1_corr", &thetaLAB);
   tree9Li->SetBranchAddress("eLoss_p1_reco", &kEnergy);
   tree9Li->SetBranchAddress("MaxR1", &MaxR);

   tree9Li->SetBranchAddress("S800_E1_dE", &s800dE);
   tree9Li->SetBranchAddress("S800_ObjCorr_ToF", &s800ToF);
   tree9Li->SetBranchAddress("S800_XfObj_ToF", &s800ToFXf);

   // Iterate over events.
   for (Int_t i = 0; i < totalEntries9Li; i++) {
      tree9Li->GetEntry(i);

      // Fill the PID histograms.

      // Apply the gates (track stop inside ATTPC and 11Li beam particle).
      if (MaxR > 250.0 - minDistanceFromBorder)
         continue;
      hs800PID->Fill(s800ToF, s800dE);

      if (300 > s800ToFXf || s800ToFXf > 340)
         continue;

      // Fill the histograms corresponding to the 9Li in the S800.
      if (cut9Li->IsInside(s800ToF, s800dE)){
         hKinematics9Li->Fill(thetaLAB, kEnergy);
         hEx9Li->Fill(Ex4);
      } else if (cut11Li->IsInside(s800ToF, s800dE)) { // Fill the histograms corresponding to th 11Li in the S800
         hKinematicsElastic->Fill(thetaLAB, kEnergy);
         hExElastic->Fill(Ex4);
      }
   }
   file9Li->Close();

   // Fit the elastic channel peak in order to obtain the resolution.
   TF1 *fElasticPeak = new TF1("fElasticPeak", "gaus(0)", -4.2, 4.2);
   Double_t paramsElastic[3] = {50, 0 , 0.05};
   fElasticPeak->SetNpx(1000);
   fElasticPeak->SetParameters(paramsElastic);
   fElasticPeak->SetParLimits(1, -1., 1.);
   fElasticPeak->SetParLimits(2, 0., 2.);
   hExElastic->Fit(fElasticPeak, "R");

   // Fit the Pygmy resonance to the convolution of a Breit-Wigner with
   // the experimental resolution from the elastic peak.
   TF1 *fPygmy = new TF1("fPygmy", "[0] * TMath::Voigt(x - [1] - [2], [3], [4])", -5, 14);
   Double_t paramsPygmy[5] = {200, 0.8, fElasticPeak->GetParameter(1), fElasticPeak->GetParameter(2), 3.};
   fPygmy->SetNpx(1000);
   fPygmy->SetParameters(paramsPygmy);
   fPygmy->SetParLimits(1, 0., 1.5);
   fPygmy->FixParameter(2, fElasticPeak->GetParameter(1));
   fPygmy->FixParameter(3, fElasticPeak->GetParameter(2));
   fPygmy->SetParLimits(4, 0., 8.);
   hEx9Li->Fit(fPygmy, "R");

   // Read the kinematic files for the kinematic histogram.
   TGraph *kineGS = ReadKinematics("kinematicLines/11Li_pp_gs.txt");
   TGraph *kinePygmyPeak = ReadKinematics("kinematicLines/11Li_pp_PygmyPeak.txt");

   // Draw the elastic channel histograms.
   TCanvas *cKinematicsElastic = new TCanvas();
   hKinematicsElastic->Draw("zcol");
   kineGS->Draw("same");
   hKinematicsElastic->GetXaxis()->SetTitle("#theta_{LAB} (deg)");
   hKinematicsElastic->GetYaxis()->SetTitle("E_{LAB} (MeV)");

   TCanvas *cExElastic = new TCanvas();
   hExElastic->Draw("E0");
   hExElastic->GetXaxis()->SetTitle("E_{ex} (MeV)");

   // Draw the 9Li gate histograms.
   TCanvas *cKinematics9Li = new TCanvas();
   hKinematics9Li->Draw("zcol");
   kinePygmyPeak->Draw("same");
   hKinematics9Li->GetXaxis()->SetTitle("#theta_{LAB} (deg)");
   hKinematics9Li->GetYaxis()->SetTitle("E_{LAB} (MeV)");

   TCanvas *cEx9Li = new TCanvas();
   hEx9Li->Draw("E0");
   hEx9Li->GetXaxis()->SetTitle("E_{ex} (MeV)");

   // Draw the PID histogram.
   TCanvas *cs800PID = new TCanvas();
   hs800PID->Draw("zcol");
   cut9Li->Draw("same");
   cut11Li->Draw("same");
   hs800PID->GetXaxis()->SetTitle("ObjToF_{S800} (?s)");
   hs800PID->GetYaxis()->SetTitle("#Delta E_{S800} (a.u.?)");
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
