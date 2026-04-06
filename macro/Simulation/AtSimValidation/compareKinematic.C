#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

#include <AtMCTrack.h>
#include <FairMCPoint.h>
#include <TCanvas.h>
#include <TClonesArray.h>
#include <TFile.h>
#include <TGraph.h>
#include <TH1D.h>
#include <TLegend.h>
#include <TMultiGraph.h>
#include <TProfile.h>
#include <TStyle.h>
#include <TTree.h>

namespace {
struct PointSample {
   double x{};
   double y{};
   double z{};
   double px{};
   double py{};
   double pz{};
   double eLoss{};
   double length{};
};

struct TrackData {
   std::vector<PointSample> points;
};

struct PlotData {
   std::vector<std::pair<double, double>> recoilProtonKinematics;
   std::vector<std::pair<double, double>> scatteredIonKinematics;
   std::vector<std::pair<double, double>> xy;
   TProfile *recoilProtonBragg{nullptr};
   TProfile *scatteredIonBragg{nullptr};
   TH1D *recoilProtonRange{nullptr};
   TH1D *scatteredIonRange{nullptr};
};

bool SortByLength(const PointSample &lhs, const PointSample &rhs) { return lhs.length < rhs.length; }

TrackData BuildTrackData(int trackID, TClonesArray *points)
{
   TrackData out;
   for (int i = 0; i < points->GetEntriesFast(); ++i) {
      auto *pt = dynamic_cast<FairMCPoint *>(points->At(i));
      if (!pt || pt->GetTrackID() != trackID)
         continue;
      out.points.push_back({pt->GetX() * 10., pt->GetY() * 10., pt->GetZ() * 10., pt->GetPx() * 1000.,
                            pt->GetPy() * 1000., pt->GetPz() * 1000., pt->GetEnergyLoss() * 1000.,
                            pt->GetLength() * 10.});
   }
   std::sort(out.points.begin(), out.points.end(), SortByLength);
   return out;
}

void FillBragg(TProfile *profile, const TrackData &track)
{
   for (size_t i = 1; i < track.points.size(); ++i) {
      const auto &prev = track.points[i - 1];
      const auto &curr = track.points[i];
      double dx = curr.x - prev.x;
      double dy = curr.y - prev.y;
      double dz = curr.z - prev.z;
      double step = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (step > 1e-6 && curr.eLoss > 0.)
         profile->Fill(curr.z, curr.eLoss / step);
   }
}

bool IsSelectedPrimaryTrack(AtMCTrack *track, int pdgCode)
{
   return track != nullptr && track->GetMotherId() == -1 && track->GetPdgCode() == pdgCode;
}

PlotData LoadPlotData(const TString &fileName, const char *tag)
{
   PlotData out;
   out.recoilProtonBragg = new TProfile(Form("hRecoilProtonBraggK_%s", tag), ";Z [mm];dE/dx [MeV/mm]", 100, 0., 1000.);
   out.scatteredIonBragg = new TProfile(Form("hScatteredIonBraggK_%s", tag), ";Z [mm];dE/dx [MeV/mm]", 100, 0., 1000.);
   out.recoilProtonRange = new TH1D(Form("hRecoilProtonRange_%s", tag), ";Stopping Z [mm];Tracks", 100, 0., 1000.);
   out.scatteredIonRange = new TH1D(Form("hScatteredIonRange_%s", tag), ";Stopping Z [mm];Tracks", 100, 0., 1000.);

   auto *file = TFile::Open(fileName);
   if (!file || file->IsZombie()) {
      std::cerr << "Cannot open " << fileName << "\n";
      return out;
   }

   auto *tree = dynamic_cast<TTree *>(file->Get("cbmsim"));
   if (!tree) {
      std::cerr << "Missing cbmsim tree in " << fileName << "\n";
      file->Close();
      return out;
   }

   TClonesArray *pointArray = nullptr;
   TClonesArray *trackArray = nullptr;
   tree->SetBranchAddress("AtTpcPoint", &pointArray);
   tree->SetBranchAddress("MCTrack", &trackArray);

   for (Long64_t iEvent = 0; iEvent < tree->GetEntries(); ++iEvent) {
      tree->GetEntry(iEvent);
      if (!pointArray || !trackArray || pointArray->GetEntriesFast() == 0)
         continue;

      int scatteredIonTrackID = -1;
      int recoilProtonTrackID = -1;
      for (int i = 0; i < trackArray->GetEntriesFast(); ++i) {
         auto *track = dynamic_cast<AtMCTrack *>(trackArray->At(i));
         if (scatteredIonTrackID < 0 && IsSelectedPrimaryTrack(track, 1000060160))
            scatteredIonTrackID = i;
         if (recoilProtonTrackID < 0 && IsSelectedPrimaryTrack(track, 2212))
            recoilProtonTrackID = i;
      }

      if (scatteredIonTrackID < 0 || recoilProtonTrackID < 0)
         continue;

      auto scatteredIon = BuildTrackData(scatteredIonTrackID, pointArray);
      auto recoilProton = BuildTrackData(recoilProtonTrackID, pointArray);
      if (scatteredIon.points.empty() || recoilProton.points.empty())
         continue;

      const auto &recoilProtonFirst = recoilProton.points.front();
      const auto &scatteredIonFirst = scatteredIon.points.front();
      out.recoilProtonKinematics.emplace_back(
         std::sqrt(recoilProtonFirst.px * recoilProtonFirst.px + recoilProtonFirst.py * recoilProtonFirst.py),
         recoilProtonFirst.pz);
      out.scatteredIonKinematics.emplace_back(
         std::sqrt(scatteredIonFirst.px * scatteredIonFirst.px + scatteredIonFirst.py * scatteredIonFirst.py),
         scatteredIonFirst.pz);

      out.recoilProtonRange->Fill(recoilProton.points.back().z);
      out.scatteredIonRange->Fill(scatteredIon.points.back().z);
      FillBragg(out.recoilProtonBragg, recoilProton);
      FillBragg(out.scatteredIonBragg, scatteredIon);

      for (const auto &point : recoilProton.points)
         out.xy.emplace_back(point.x, point.y);
      for (const auto &point : scatteredIon.points)
         out.xy.emplace_back(point.x, point.y);
   }

   file->Close();
   return out;
}

void StyleProfile(TProfile *hist, Color_t color, Style_t style)
{
   hist->SetLineColor(color);
   hist->SetLineWidth(2);
   hist->SetLineStyle(style);
}

void StyleRange(TH1D *hist, Color_t color, Style_t style)
{
   hist->SetLineColor(color);
   hist->SetLineWidth(2);
   hist->SetLineStyle(style);
}

TGraph *MakeGraph(const std::vector<std::pair<double, double>> &points, const char *name, Color_t color)
{
   auto *graph = new TGraph(points.size());
   graph->SetName(name);
   graph->SetMarkerStyle(20);
   graph->SetMarkerSize(0.45);
   graph->SetMarkerColor(color);
   graph->SetLineColor(color);
   for (size_t i = 0; i < points.size(); ++i)
      graph->SetPoint(i, points[i].first, points[i].second);
   return graph;
}
} // namespace

void compareKinematic(TString geantFile = "./data/geant4_kinematic.root",
                      TString simpleFile = "./data/simpleSim_kinematic.root")
{
   gStyle->SetOptStat(0);
   auto geant = LoadPlotData(geantFile, "g4");
   auto simple = LoadPlotData(simpleFile, "sim");

   StyleProfile(geant.recoilProtonBragg, kBlue + 1, 1);
   StyleProfile(simple.recoilProtonBragg, kRed + 1, 2);
   StyleProfile(geant.scatteredIonBragg, kBlue + 1, 1);
   StyleProfile(simple.scatteredIonBragg, kRed + 1, 2);
   StyleRange(geant.recoilProtonRange, kBlue + 1, 1);
   StyleRange(simple.recoilProtonRange, kRed + 1, 2);
   StyleRange(geant.scatteredIonRange, kBlue + 1, 3);
   StyleRange(simple.scatteredIonRange, kRed + 1, 4);

   auto *protonG4 = MakeGraph(geant.recoilProtonKinematics, "protonG4", kBlue + 1);
   auto *protonSim = MakeGraph(simple.recoilProtonKinematics, "protonSim", kRed + 1);
   auto *ionG4 = MakeGraph(geant.scatteredIonKinematics, "ionG4", kBlue + 1);
   auto *ionSim = MakeGraph(simple.scatteredIonKinematics, "ionSim", kRed + 1);
   auto *xyG4 = MakeGraph(geant.xy, "xyG4K", kBlue + 1);
   auto *xySim = MakeGraph(simple.xy, "xySimK", kRed + 1);

   auto *canvas = new TCanvas("cKinematicCompare", "AtSimpleSimulation kinematic validation", 1600, 900);
   canvas->Divide(3, 2);

   canvas->cd(1);
   auto *protonMg = new TMultiGraph();
   protonMg->SetTitle("Proton kinematic locus;p_{T} [MeV/c];p_{Z} [MeV/c]");
   protonMg->Add(protonG4, "P");
   protonMg->Add(protonSim, "P");
   protonMg->Draw("A");
   auto *protonLegend = new TLegend(0.62, 0.78, 0.9, 0.9);
   protonLegend->AddEntry(protonG4, "Geant4", "p");
   protonLegend->AddEntry(protonSim, "SimpleSim", "p");
   protonLegend->Draw();

   canvas->cd(2);
   auto *ionMg = new TMultiGraph();
   ionMg->SetTitle("Scattered 16C kinematic locus;p_{T} [MeV/c];p_{Z} [MeV/c]");
   ionMg->Add(ionG4, "P");
   ionMg->Add(ionSim, "P");
   ionMg->Draw("A");
   auto *ionLegend = new TLegend(0.62, 0.78, 0.9, 0.9);
   ionLegend->AddEntry(ionG4, "Geant4", "p");
   ionLegend->AddEntry(ionSim, "SimpleSim", "p");
   ionLegend->Draw();

   canvas->cd(3);
   geant.recoilProtonBragg->SetTitle("Bragg curve: recoil proton");
   geant.recoilProtonBragg->Draw("hist");
   simple.recoilProtonBragg->Draw("hist same");
   auto *braggProtonLegend = new TLegend(0.62, 0.78, 0.9, 0.9);
   braggProtonLegend->AddEntry(geant.recoilProtonBragg, "Geant4", "l");
   braggProtonLegend->AddEntry(simple.recoilProtonBragg, "SimpleSim", "l");
   braggProtonLegend->Draw();

   canvas->cd(4);
   geant.scatteredIonBragg->SetTitle("Bragg curve: scattered 16C");
   geant.scatteredIonBragg->Draw("hist");
   simple.scatteredIonBragg->Draw("hist same");
   auto *braggIonLegend = new TLegend(0.62, 0.78, 0.9, 0.9);
   braggIonLegend->AddEntry(geant.scatteredIonBragg, "Geant4", "l");
   braggIonLegend->AddEntry(simple.scatteredIonBragg, "SimpleSim", "l");
   braggIonLegend->Draw();

   canvas->cd(5);
   geant.recoilProtonRange->SetTitle("Range distributions");
   geant.recoilProtonRange->Draw("hist");
   simple.recoilProtonRange->Draw("hist same");
   geant.scatteredIonRange->Draw("hist same");
   simple.scatteredIonRange->Draw("hist same");
   auto *rangeLegend = new TLegend(0.44, 0.64, 0.9, 0.9);
   rangeLegend->AddEntry(geant.recoilProtonRange, "Geant4 proton", "l");
   rangeLegend->AddEntry(simple.recoilProtonRange, "SimpleSim proton", "l");
   rangeLegend->AddEntry(geant.scatteredIonRange, "Geant4 16C", "l");
   rangeLegend->AddEntry(simple.scatteredIonRange, "SimpleSim 16C", "l");
   rangeLegend->Draw();

   canvas->cd(6);
   auto *xyMg = new TMultiGraph();
   xyMg->SetTitle("XY projection;X [mm];Y [mm]");
   xyMg->Add(xyG4, "P");
   xyMg->Add(xySim, "P");
   xyMg->Draw("A");
   auto *xyLegend = new TLegend(0.65, 0.78, 0.9, 0.9);
   xyLegend->AddEntry(xyG4, "Geant4", "p");
   xyLegend->AddEntry(xySim, "SimpleSim", "p");
   xyLegend->Draw();

   gSystem->mkdir("data", kTRUE);
   canvas->SaveAs("./data/compareKinematic.pdf");
}
