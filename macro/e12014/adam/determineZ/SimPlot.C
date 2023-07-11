/**
 * Tools for making simulated angle vs Z plots under different assumptions
 */
TChain *tree = nullptr;
TH1F *hFold = nullptr;
TH1F *hAng = nullptr;
TH1F *hFoldLab = nullptr;
TH2F *hAngvsZ = nullptr;
TH1F *hAngCut = nullptr;
TH1F *hAngCut2 = nullptr;
TH1F *hAngCut3 = nullptr;

TH2F *hAngvsZTrig = nullptr;
TH1 *hZ = nullptr;
TH1 *hZTrig = nullptr;

using VecXYZE = ROOT::Math::LorentzVector<ROOT::Math::PxPyPzE4D<>>;
using test = AtTools::AtKinematics; // Literally this is just to force ROOT to load the AtTools lib

bool UseRelBoost = true;

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

bool cut1(double z, double ang)
{
   auto vZ = z;
   if (vZ < 200 || vZ > 955)
      return false;

   // Bi200
   double upper = 0.58 + (955 - vZ) * .2 / 755;
   double lower = 0.46 + (955 - vZ) * .2 / 755;

   return ang < upper && ang > lower;
}
bool cut2(double z, double ang)
{
   // Get the upper angle cut
   auto vZ = z;
   if (vZ < 200 || vZ > 955)
      return false;

   // Bi200
   double upper = 0.58 + (955 - vZ) * .2 / 755;
   upper = 0.46 + (955 - vZ) * .2 / 755;

   double lower = 0.40 + (955 - vZ) * .2 / 755;

   return ang < upper && ang > lower;
}
bool cut3(double z, double ang)
{
   return true;
   // Get the upper angle cut
   auto vZ = z;
   if (vZ < 200 || vZ > 955)
      return false;

   // Bi200
   double upper = 0.58 + (955 - vZ) * .2 / 755;
   double lower = 0.40 + (955 - vZ) * .2 / 755;

   return ang < upper && ang > lower;
}

double getVRatio(double angDecay, double v_cm, double v_b)
{
   double ratio2 = (v_b * v_b + v_cm * v_cm) / (v_b * v_b + v_cm * v_cm + 2 * v_b * v_cm * cos(angDecay));
   return sqrt(ratio2);
}

void PlotStopingRatio()
{

   // Start by creating all of the energy loss models we will use
   std::map<int, std::shared_ptr<AtTools::AtELossTable>> lossModels; //[z] = model

   int Zcn = 83 + 2;
   int Acn = 200 + 4;
   for (int i = 30; i <= 55; i++) {
      int Z = i;
      int A = std::round((double)i / Zcn * Acn);
      auto eloss = std::make_shared<AtTools::AtELossTable>();
      eloss->LoadSrimTable(TString::Format("./eLoss/SRIM/%d_%d.txt", Z, A).Data());
      lossModels[Z] = eloss;
   }

   // Output is CSV file with column 50/50 55/45 60/40 65/35 split and rows 90 60 30 0 decay angle.
   ofstream oFile("out.csv");
   if (!oFile.is_open())
      return;

   for (int angle = 150; angle >= 30; angle -= 30) {
      for (int mass = 50; mass <= 65; mass += 5) {
         // Open the root file
         TChain tree("fragments");
         tree.Add(
            TString::Format("/user/anthonya/attpcroot/macro/e12014/simulation/eventGenerator/%d-%d.root", mass, angle));

         TTreeReader reader(&tree);
         TTreeReaderValue<std::vector<VecXYZE>> fragArray(reader, "decayFragments");
         TTreeReaderValue<std::vector<Int_t>> chargeArray(reader, "Z");

         // We only need a single event
         reader.Next();

         auto p1 = (*fragArray)[0].Vect();
         auto p2 = (*fragArray)[1].Vect();

         auto z = gRandom->Uniform(1000);
         auto beam = GetBeam(z);
         auto boost = ROOT::Math::Boost(beam.BoostToCM());

         auto vec1 = boost((*fragArray)[0]);
         auto vec2 = boost((*fragArray)[1]);

         // Now look at the location where we hit the pad plane
         XYZPoint fWindow = {10, 0, 0};
         XYZPoint fPadPlane = {0, -6, 1000};
         XYZPoint beamLoc = fWindow + (fPadPlane - fWindow) / (fPadPlane.Z() - fWindow.Z()) * z;
         XYZPoint locPP1 = beamLoc + vec1.Vect() / vec1.Vect().z() * (1000 - z);
         XYZPoint locPP2 = beamLoc + vec2.Vect() / vec2.Vect().z() * (1000 - z);

         LOG(info) << "z: " << z << " beam: " << beamLoc;
         LOG(info) << " vec1: " << vec1.Vect() << " pp1: " << locPP1;
         LOG(info) << " vec1: " << vec2.Vect() << " pp1: " << locPP2;

         ROOT::Math::XYPoint pad1 = {locPP1.x(), locPP1.y()};
         ROOT::Math::XYPoint pad2 = {locPP2.x(), locPP2.y()};
         bool passedCut = E12014::fMap->IsInhibited(E12014::fMap->GetPadNum(pad1)) != AtMap::InhibitType::kLowGain;
         passedCut &= E12014::fMap->IsInhibited(E12014::fMap->GetPadNum(pad2)) != AtMap::InhibitType::kLowGain;

         // E1 is the heavier fragment, by definition in the simulation.
         int z1 = chargeArray->at(0);
         int z2 = chargeArray->at(1);

         double E1 = vec1.E() - vec1.M();
         double E2 = vec2.E() - vec2.M();

         /*
             if (E2 > E1) {
                     swap(E1, E2);
                     swap(z1, z2);
                  }
         */
         double ratio = lossModels[z2]->GetdEdx(E2) / lossModels[z1]->GetdEdx(E1);
         // double ratio = lossModels[z2]->GetEnergyLoss(E2, 500) / lossModels[z1]->GetEnergyLoss(E1, 500);
         std::cout << z2 << "/" << z1 << " " << angle << " " << E2 << "/" << E1 << " " << lossModels[z2]->GetdEdx(E2)
                   << "/" << lossModels[z1]->GetdEdx(E1) << " = " << ratio << endl;
         oFile << ratio << ",";
      }
      oFile << endl;
   }
}

void SimPlot()
{
   E12014::CreateMap();

   // TString fileName = "/user/anthonya/attpcroot/macro/e12014/simulation/eventGenerator/fissionFragments.root";
   TString fileName = "/user/anthonya/attpcroot/macro/e12014/simulation/eventGenerator/fissionFragments.root";

   if (!tree) {
      tree = new TChain("fragments");
      tree->Add(fileName);
   }

   hFold = new TH1F("hFold", "Folding Angle", 50, 0, TMath::Pi() + 1e-4);
   hAng = new TH1F("hAng", "Decay angle (w.r.t. z)", 50, 0, TMath::Pi());
   hAngCut = new TH1F("hAngCut", "Decay angle (w.r.t. z)", 50, 0, 180);
   hAngCut2 = new TH1F("hAngCut2", "sin(Decay angle) (w.r.t. z)", 50, 0.75, 1);
   hAngCut3 = new TH1F("hAngCut3", "v_actual/v_assumed (lab frame)", 50, 0.6, 1.8);
   hFoldLab = new TH1F("hFoldAngle", "Folding Angle Lab", 50, 0, TMath::Pi() + 1e-4);
   hAngvsZ = new TH2F("hAngVsLoc", "Folding angle vs Z", 50, 00, 1000, 50, 0, 1);
   hAngvsZTrig = new TH2F("hAngvsZTrig", "Folding angle vs Z Triggered", 50, 00, 1000, 50, 0, 1);

   TTreeReader reader(tree);
   TTreeReaderValue<std::vector<VecXYZE>> fragArray(reader, "decayFragments");
   TTreeReaderValue<std::vector<Int_t>> massArray(reader, "A");

   while (reader.Next() && reader.GetCurrentEntry() < 100000000) {
      auto p1 = (*fragArray)[0].Vect();
      auto p2 = (*fragArray)[1].Vect();

      auto zDir = ROOT::Math::XYZVector(0, 0, 1);
      auto foldingAngle = ROOT::Math::VectorUtil::Angle(p1, p2);
      auto decayAngle = ROOT::Math::VectorUtil::Angle(p1, zDir);
      auto decayAngle2 = ROOT::Math::VectorUtil::Angle(p2, zDir);

      auto v_cm1 = AtTools::Kinematics::GetBeta(p1.R(), (*massArray)[0]);
      auto v_cm2 = AtTools::Kinematics::GetBeta(p2.R(), (*massArray)[1]);

      hFold->Fill(foldingAngle);
      hAng->Fill(decayAngle);
      hAng->Fill(decayAngle2);

      // Get the beam boost
      auto z = gRandom->Uniform(1000);
      auto beam = GetBeam(z);
      auto boost = ROOT::Math::Boost(beam.BoostToCM());

      auto p1T =
         boost(AtTools::Kinematics::Get4Vector(XYZVector(p1.R(), 0, 0), AtTools::Kinematics::AtoE((*massArray)[0])));
      auto p2T =
         boost(AtTools::Kinematics::Get4Vector(XYZVector(p2.R(), 0, 0), AtTools::Kinematics::AtoE((*massArray)[1])));

      double v_1T, v_2T, v_1, v_2;
      if (UseRelBoost) {
         v_1T = AtTools::Kinematics::GetBeta(p1T.R(), (*massArray)[0]);
         v_2T = AtTools::Kinematics::GetBeta(p2T.R(), (*massArray)[1]);
      } else {
         v_1T = sqrt(v_cm1 * v_cm1 + beam.Beta() * beam.Beta());
         v_2T = sqrt(v_cm2 * v_cm2 + beam.Beta() * beam.Beta());
      }

      p1 = boost((*fragArray)[0]).Vect();
      p2 = boost((*fragArray)[1]).Vect();

      //** Plot folding angle vs Z and the cut folding angle vs Z
      hAngvsZ->Fill(1000 - z, ROOT::Math::VectorUtil::Angle(p1, p2));

      XYZPoint fWindow = {10, 0, 0};
      XYZPoint fPadPlane = {0, -6, 1000};
      XYZPoint beamLoc = fWindow + (fPadPlane - fWindow) / (fPadPlane.Z() - fWindow.Z()) * z;
      XYZPoint locPP1 = beamLoc + p1 / p1.z() * (1000 - z);
      XYZPoint locPP2 = beamLoc + p2 / p2.z() * (1000 - z);

      ROOT::Math::XYPoint pad1 = {locPP1.x(), locPP1.y()};
      ROOT::Math::XYPoint pad2 = {locPP2.x(), locPP2.y()};
      bool passedCut = E12014::fMap->IsInhibited(E12014::fMap->GetPadNum(pad1)) != AtMap::InhibitType::kLowGain;
      passedCut &= E12014::fMap->IsInhibited(E12014::fMap->GetPadNum(pad2)) != AtMap::InhibitType::kLowGain;

      double rCut = 90;
      passedCut = pad1.R() > rCut;
      passedCut &= pad2.R() > rCut;
      if (passedCut)
         hAngvsZTrig->Fill(1000 - z, ROOT::Math::VectorUtil::Angle(p1, p2));

      if (UseRelBoost) {
         v_1 = AtTools::Kinematics::GetBeta(p1.R(), (*massArray)[0]);
         v_2 = AtTools::Kinematics::GetBeta(p2.R(), (*massArray)[1]);
      } else {
         v_1 = sqrt(v_cm1 * v_cm1 + beam.Beta() * beam.Beta() + 2 * v_cm1 * beam.Beta() * cos(decayAngle));
         v_2 = sqrt(v_cm2 * v_cm2 + beam.Beta() * beam.Beta() + 2 * v_cm2 * beam.Beta() * cos(decayAngle2));
      }

      foldingAngle = ROOT::Math::VectorUtil::Angle(p1, p2);
      hFoldLab->Fill(foldingAngle);

      if (cut1(1000 - z, foldingAngle)) {
         hAngCut->Fill(decayAngle * TMath::RadToDeg());
         hAngCut->Fill(decayAngle2 * TMath::RadToDeg());

         hAngCut2->Fill(sin(decayAngle));
         hAngCut2->Fill(sin(decayAngle2));

         // if (decayAngle > TMath::Pi() / 2)
         hAngCut3->Fill(v_1T / v_1);
         // else
         hAngCut3->Fill(v_2T / v_2);
         // hAngvsZ->Fill(1000 - z, ROOT::Math::VectorUtil::Angle(p1, p2));
      }
   }

   hZ = hAngvsZ->ProjectionX();
   hZTrig = hAngvsZTrig->ProjectionX();
   hAng->Draw();
}
