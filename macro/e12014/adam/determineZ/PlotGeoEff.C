
TH2F *hAngvsZ = nullptr;
TH2F *hAngvsZTrig = nullptr;
TH2F *hAngvsZEff = nullptr;

TH2F *hAngvsZRaw = nullptr;
TH2F *hAngvsZCorr = nullptr;

int binNum = 45;

using XYZPoint = ROOT::Math::XYZPoint;
using VecXYZE = ROOT::Math::LorentzVector<ROOT::Math::PxPyPzE4D<>>;
using test = AtTools::AtKinematics; // Literally this is just to force ROOT to load the AtTools lib

double GetBeamKE(double zLoc)
{
   return 2713.28 + zLoc * -2.30434 + zLoc * zLoc * 0.000455263;
}
VecXYZE GetBeam(double zLoc)
{
   double KE = GetBeamKE(zLoc);

   double m = 204 * 931.5;
   double E = KE + m;

   double p = sqrt(E * E - m * m);

   return {0, 0, -p, E};
   // p = mv gamma
}

// Plot the detection efficiecy keeping fission events where both fission fragments hit the pad plane
// at a radius greater than rCut (mm)
void PlotGeoEff(double rCut = 100)
{

   TChain tree("fragments");
   tree.Add("/user/anthonya/attpcroot/macro/e12014/simulation/eventGenerator/fissionFragmentsUniform.root");

   hAngvsZ = new TH2F("hAngVsLoc", "Folding angle vs Z", binNum, 00, 900, binNum, 0, 1);
   hAngvsZTrig = new TH2F("hAngVsLocTrig", "Folding angle vs Z Triggered", binNum, 00, 900, binNum, 0, 1);
   hAngvsZEff = new TH2F("hAngVsLocEff", "Folding angle vs Z Eff", binNum, 00, 900, binNum, 0, 1);

   TTreeReader reader(&tree);
   TTreeReaderValue<std::vector<VecXYZE>> fragArray(reader, "decayFragments");
   TTreeReaderValue<std::vector<Int_t>> massArray(reader, "A");

   while (reader.Next()) {

      auto p1 = (*fragArray)[0].Vect();
      auto p2 = (*fragArray)[1].Vect();

      auto zDir = ROOT::Math::XYZVector(0, 0, 1);

      auto z = gRandom->Uniform(1000);
      auto beam = GetBeam(z);
      auto boost = ROOT::Math::Boost(beam.BoostToCM());

      p1 = boost((*fragArray)[0]).Vect();
      p2 = boost((*fragArray)[1]).Vect();

      double foldingAngle = ROOT::Math::VectorUtil::Angle(p1, p2);

      //** Plot folding angle vs Z and the cut folding angle vs Z
      hAngvsZ->Fill(1000 - z, foldingAngle);

      XYZPoint fWindow = {10, 0, 0};
      XYZPoint fPadPlane = {0, -6, 1000};
      XYZPoint beamLoc = fWindow + (fPadPlane - fWindow) / (fPadPlane.Z() - fWindow.Z()) * z;
      XYZPoint locPP1 = beamLoc + p1 / p1.z() * (1000 - z);
      XYZPoint locPP2 = beamLoc + p2 / p2.z() * (1000 - z);

      ROOT::Math::XYPoint pad1 = {locPP1.x(), locPP1.y()};
      ROOT::Math::XYPoint pad2 = {locPP2.x(), locPP2.y()};

      double rCut = 100;
      bool passedCut = pad1.R() > rCut && pad2.R() > rCut;
      if (passedCut)
         hAngvsZTrig->Fill(1000 - z, foldingAngle);
   }

   for (int i = 0; i < binNum; ++i)
      for (int j = 0; j < binNum; ++j) {
         double eff = hAngvsZTrig->GetBinContent(i + 1, j + 1) / hAngvsZ->GetBinContent(i + 1, j + 1);
         if (eff > 0.15)
            hAngvsZEff->SetBinContent(i + 1, j + 1, eff);
      }
}

// Fill 2D histograms of angle vs Z for fission events. One raw and the other corrected for the
// efficiecny in hAngVsZEff.
void PlotCounts()
{

   TChain tree("cbmsim");
   tree.Add("/mnt/analysis/e12014/TPC/150Torr_yFit/Bi200.root");

   hAngvsZRaw = new TH2F("hAngVsRaw", "Folding angle vs Z Exp Raw", binNum, 00, 900, binNum, 0, 1);
   hAngvsZCorr = new TH2F("hAngVsCor", "Folding angle vs Z Exp Corr", binNum, 00, 900, binNum, 0, 1);

   tree.Draw("AtFissionEvent->GetFoldingAngle():AtFissionEvent->GetVertex().z()>>hAngVsRaw");

   for (int i = 0; i < binNum; ++i)
      for (int j = 0; j < binNum; ++j) {
         auto eff = hAngvsZEff->GetBinContent(i + 1, j + 1);
         if (eff != 0)
            hAngvsZCorr->SetBinContent(i + 1, j + 1, hAngvsZRaw->GetBinContent(i + 1, j + 1) / eff);
         // hAngvsZCorr->SetBinError(i + 1, j + 1, sqrt(hAngvsZRaw->GetBinContent(i + 1, j + 1)) / eff);
      }
}
