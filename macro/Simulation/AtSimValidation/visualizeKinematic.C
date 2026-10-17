#include "../../Kinematics/Decay_kinematics/TRelativisticKinematics.hh"
#include "../../Kinematics/Decay_kinematics/TRelativisticKinematics.cxx"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <map>
#include <utility>
#include <vector>

#include <TCanvas.h>
#include <TClonesArray.h>
#include <TFile.h>
#include <TGraph.h>
#include <TLegend.h>
#include <TMultiGraph.h>
#include <TPolyLine3D.h>
#include <TStyle.h>
#include <TTree.h>

namespace {
struct PointSample {
   double x{};
   double y{};
   double z{};
   double length{};
   double px{};
   double py{};
   double pz{};
};

bool SortByLength(const PointSample &lhs, const PointSample &rhs) { return lhs.length < rhs.length; }

const char *GetTrackLabel(Int_t trackID)
{
   if (trackID == 1)
      return "scattered ion";
   if (trackID == 2)
      return "recoil proton";
   return "selected track";
}

bool GetReactionConfig(Int_t trackID, double &massMeV, double &m3Amu, double &m4Amu)
{
   constexpr double c16MassAmu = 16.014701;
   constexpr double protonMassAmu = 1.0078250322;
   constexpr double u = 931.49401;

   if (trackID == 1) {
      massMeV = c16MassAmu * u;
      m3Amu = c16MassAmu;
      m4Amu = protonMassAmu;
      return true;
   }

   if (trackID == 2) {
      massMeV = protonMassAmu * u;
      m3Amu = c16MassAmu;
      m4Amu = protonMassAmu;
      return true;
   }

   return false;
}

bool IsSelectedPrimaryTrack(Int_t selectedTrackID, AtMCTrack *track)
{
   if (track == nullptr || track->GetMotherId() != -1)
      return false;

   if (selectedTrackID == 1)
      return track->GetPdgCode() == 1000060160;
   if (selectedTrackID == 2)
      return track->GetPdgCode() == 2212;

   return false;
}

TGraph *BuildTheoryGraph(Int_t trackID)
{
   constexpr double c16MassAmu = 16.014701;
   constexpr double protonMassAmu = 1.0078250322;
   constexpr double beamKineticEnergyMeV = 175.0;

   double massMeV = 0.0;
   double m3Amu = 0.0;
   double m4Amu = 0.0;
   if (!GetReactionConfig(trackID, massMeV, m3Amu, m4Amu))
      return nullptr;

   TRelativisticKinematics kine;
   kine.SetMassOfProjectile(c16MassAmu);
   kine.SetMassOfTarget(protonMassAmu);
   kine.SetMassOfScattered(m3Amu);
   kine.SetMassOfRecoiled(m4Amu);
   kine.SetExEnergyOfProjectile(0.0);
   kine.SetExEnergyOfTarget(0.0);
   kine.SetExEnergyOfScattered(0.0);
   kine.SetExEnergyOfRecoiled(0.0);
   kine.SetLabEnergy(beamKineticEnergyMeV);

   std::vector<std::pair<double, double>> theoryPoints;
   theoryPoints.reserve(1801);

   std::ostringstream sink;
   auto *oldBuf = std::cout.rdbuf(sink.rdbuf());
   for (double thetaCm = 0.0; thetaCm <= 180.0; thetaCm += 0.1) {
      kine.SetThetaCMAngle(thetaCm);
      kine.Kinematics();

      double thetaLab = 0.0;
      double energyLab = 0.0;
      if (trackID == 1) {
         thetaLab = kine.GetANGAs(0) * TMath::RadToDeg();
         energyLab = kine.GetANGAs(1);
      } else {
         thetaLab = kine.GetANGAr(0) * TMath::RadToDeg();
         energyLab = kine.GetANGAr(1);
      }

      if (std::isfinite(thetaLab) && std::isfinite(energyLab))
         theoryPoints.emplace_back(thetaLab, energyLab);
   }
   std::cout.rdbuf(oldBuf);

   if (theoryPoints.empty())
      return nullptr;

   auto *graph = new TGraph(theoryPoints.size());
   graph->SetName("gTheoryKinematics");
   graph->SetLineColor(kRed + 1);
   graph->SetLineWidth(3);
   for (size_t i = 0; i < theoryPoints.size(); ++i)
      graph->SetPoint(i, theoryPoints[i].first, theoryPoints[i].second);

   return graph;
}
} // namespace

void visualizeKinematic(TString inputFile = "./data/simpleSim_kinematic.root", Int_t selectedTrackID = 2, Int_t maxEvents = 8)
{
   gStyle->SetOptStat(0);

   auto *file = TFile::Open(inputFile);
   if (!file || file->IsZombie()) {
      std::cerr << "Cannot open " << inputFile << "\n";
      return;
   }

   auto *tree = dynamic_cast<TTree *>(file->Get("cbmsim"));
   if (!tree) {
      std::cerr << "Missing cbmsim tree in " << inputFile << "\n";
      file->Close();
      return;
   }

   TClonesArray *pointArray = nullptr;
   TClonesArray *trackArray = nullptr;
   tree->SetBranchAddress("AtTpcPoint", &pointArray);
   tree->SetBranchAddress("MCTrack", &trackArray);

   auto *xyCanvas = new TCanvas("cKinematicXY", "Kinematic XY", 900, 700);
   auto *xyFrame = xyCanvas->DrawFrame(-300., -300., 300., 300.,
                                       Form("XY projection: %s;X [mm];Y [mm]", GetTrackLabel(selectedTrackID)));
   xyFrame->GetYaxis()->SetTitleOffset(1.2);

   auto *trackCanvas = new TCanvas("cKinematic3D", "Kinematic 3D", 1000, 800);
   trackCanvas->cd();

   auto *kineCanvas = new TCanvas("cKinematicLine", "Kinematic line", 900, 700);

   auto *legend = new TLegend(0.68, 0.72, 0.92, 0.9);
   bool first3D = true;
   bool firstXY = true;
   int drawnEvents = 0;
   std::vector<std::pair<double, double>> measuredKinematics;

   for (Long64_t iEvent = 0; iEvent < tree->GetEntries(); ++iEvent) {
      tree->GetEntry(iEvent);
      if (!pointArray || !trackArray || pointArray->GetEntriesFast() < 2)
         continue;

      std::map<int, std::vector<PointSample>> tracks;
      for (int i = 0; i < pointArray->GetEntriesFast(); ++i) {
         auto *pt = dynamic_cast<FairMCPoint *>(pointArray->At(i));
         if (!pt)
            continue;
         tracks[pt->GetTrackID()].push_back(
            {pt->GetX() * 10., pt->GetY() * 10., pt->GetZ() * 10., pt->GetLength() * 10., pt->GetPx() * 1000.,
             pt->GetPy() * 1000., pt->GetPz() * 1000.});
      }

      int selectedEventTrackID = -1;
      for (int i = 0; i < trackArray->GetEntriesFast(); ++i) {
         auto *track = dynamic_cast<AtMCTrack *>(trackArray->At(i));
         if (IsSelectedPrimaryTrack(selectedTrackID, track)) {
            selectedEventTrackID = i;
            break;
         }
      }
      if (selectedEventTrackID < 0)
         continue;

      auto it = tracks.find(selectedEventTrackID);
      if (it == tracks.end())
         continue;

      auto &points = it->second;
      std::sort(points.begin(), points.end(), SortByLength);
      if (points.empty())
         continue;

      if (drawnEvents < maxEvents) {
         int color = kBlue + drawnEvents % 6;

         auto *line = new TPolyLine3D(points.size());
         line->SetLineColor(color);
         line->SetLineWidth(2);

         auto *xy = new TGraph(points.size());
         xy->SetMarkerStyle(20);
         xy->SetMarkerSize(0.35);
         xy->SetMarkerColor(color);
         xy->SetLineColor(color);

         for (size_t i = 0; i < points.size(); ++i) {
            line->SetPoint(i, points[i].x, points[i].y, points[i].z);
            xy->SetPoint(i, points[i].x, points[i].y);
         }

         trackCanvas->cd();
         if (first3D) {
            line->Draw();
            first3D = false;
         } else {
            line->Draw("same");
         }

         xyCanvas->cd();
         if (firstXY) {
            xy->Draw("PL");
            firstXY = false;
         } else {
            xy->Draw("PL same");
         }

         legend->AddEntry(line, Form("Reaction event %lld", iEvent), "l");
         ++drawnEvents;
      }

      double massMeV = 0.0;
      double m3Amu = 0.0;
      double m4Amu = 0.0;
      if (!GetReactionConfig(selectedTrackID, massMeV, m3Amu, m4Amu))
         continue;
      (void)m3Amu;
      (void)m4Amu;

      const auto &firstPoint = points.front();
      const double p2 = firstPoint.px * firstPoint.px + firstPoint.py * firstPoint.py + firstPoint.pz * firstPoint.pz;
      const double thetaDeg = std::atan2(std::sqrt(firstPoint.px * firstPoint.px + firstPoint.py * firstPoint.py),
                                         firstPoint.pz) *
                              TMath::RadToDeg();
      const double keMeV = std::sqrt(p2 + massMeV * massMeV) - massMeV;
      measuredKinematics.emplace_back(thetaDeg, keMeV);
   }

   auto *measured = new TGraph(measuredKinematics.size());
   measured->SetName("gMeasuredKinematics");
   measured->SetMarkerStyle(20);
   measured->SetMarkerSize(0.9);
   measured->SetMarkerColor(kBlue + 1);
   measured->SetLineColor(kBlue + 1);
   for (size_t i = 0; i < measuredKinematics.size(); ++i)
      measured->SetPoint(i, measuredKinematics[i].first, measuredKinematics[i].second);

   auto *theory = BuildTheoryGraph(selectedTrackID);

   trackCanvas->cd();
   legend->Draw();
   trackCanvas->Modified();
   trackCanvas->Update();

   xyCanvas->Modified();
   xyCanvas->Update();

   kineCanvas->cd();
   auto *mg = new TMultiGraph();
   mg->SetTitle(Form("KE vs #theta_{lab}: %s;#theta_{lab} [deg];KE [MeV]", GetTrackLabel(selectedTrackID)));
   if (theory)
      mg->Add(theory, "L");
   mg->Add(measured, "P");
   mg->Draw("A");
   auto *kineLegend = new TLegend(0.62, 0.76, 0.9, 0.9);
   kineLegend->AddEntry(measured, "Simulation", "p");
   if (theory)
      kineLegend->AddEntry(theory, "Theory", "l");
   kineLegend->Draw();
   kineCanvas->Modified();
   kineCanvas->Update();

   std::cout << "Visualized track " << selectedTrackID << " (" << GetTrackLabel(selectedTrackID) << ") from "
             << inputFile << "\n";
   std::cout << "Drew " << drawnEvents << " trajectories and " << measuredKinematics.size() << " event points\n";
}
