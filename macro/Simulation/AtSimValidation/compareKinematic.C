#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <utility>
#include <vector>

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
   std::vector<std::pair<double, double>> protonKinematics;
   std::vector<std::pair<double, double>> heliumKinematics;
   std::vector<std::pair<double, double>> xy;
   TProfile *protonBragg{nullptr};
   TProfile *heliumBragg{nullptr};
   TH1D *protonRange{nullptr};
   TH1D *heliumRange{nullptr};
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

PlotData LoadPlotData(const TString &fileName, const char *tag)
{
   PlotData out;
   out.protonBragg = new TProfile(Form("hProtonBraggK_%s", tag), ";Z [mm];dE/dx [MeV/mm]", 100, 0., 1000.);
   out.heliumBragg = new TProfile(Form("hHeliumBraggK_%s", tag), ";Z [mm];dE/dx [MeV/mm]", 100, 0., 1000.);
   out.protonRange = new TH1D(Form("hProtonRange_%s", tag), ";Stopping Z [mm];Tracks", 100, 0., 1000.);
   out.heliumRange = new TH1D(Form("hHeliumRange_%s", tag), ";Stopping Z [mm];Tracks", 100, 0., 1000.);

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
   tree->SetBranchAddress("AtTpcPoint", &pointArray);

   for (Long64_t iEvent = 0; iEvent < tree->GetEntries(); ++iEvent) {
      tree->GetEntry(iEvent);
      if (!pointArray || pointArray->GetEntriesFast() == 0)
         continue;

      std::vector<int> trackIDs;
      std::map<int, bool> seenTrack;
      for (int i = 0; i < pointArray->GetEntriesFast(); ++i) {
         auto *pt = dynamic_cast<FairMCPoint *>(pointArray->At(i));
         if (!pt)
            continue;
         if (!seenTrack[pt->GetTrackID()]) {
            seenTrack[pt->GetTrackID()] = true;
            trackIDs.push_back(pt->GetTrackID());
         }
      }

      std::sort(trackIDs.begin(), trackIDs.end());
      if (trackIDs.size() < 2)
         continue;

      auto proton = BuildTrackData(trackIDs.front(), pointArray);
      auto helium = BuildTrackData(trackIDs.back(), pointArray);
      if (proton.points.empty() || helium.points.empty())
         continue;

      const auto &protonFirst = proton.points.front();
      const auto &heliumFirst = helium.points.front();
      out.protonKinematics.emplace_back(std::sqrt(protonFirst.px * protonFirst.px + protonFirst.py * protonFirst.py),
                                        protonFirst.pz);
      out.heliumKinematics.emplace_back(std::sqrt(heliumFirst.px * heliumFirst.px + heliumFirst.py * heliumFirst.py),
                                        heliumFirst.pz);

      out.protonRange->Fill(proton.points.back().z);
      out.heliumRange->Fill(helium.points.back().z);
      FillBragg(out.protonBragg, proton);
      FillBragg(out.heliumBragg, helium);

      for (const auto &point : proton.points)
         out.xy.emplace_back(point.x, point.y);
      for (const auto &point : helium.points)
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

   StyleProfile(geant.protonBragg, kBlue + 1, 1);
   StyleProfile(simple.protonBragg, kRed + 1, 2);
   StyleProfile(geant.heliumBragg, kBlue + 1, 1);
   StyleProfile(simple.heliumBragg, kRed + 1, 2);
   StyleRange(geant.protonRange, kBlue + 1, 1);
   StyleRange(simple.protonRange, kRed + 1, 2);
   StyleRange(geant.heliumRange, kBlue + 1, 3);
   StyleRange(simple.heliumRange, kRed + 1, 4);

   auto *protonG4 = MakeGraph(geant.protonKinematics, "protonG4", kBlue + 1);
   auto *protonSim = MakeGraph(simple.protonKinematics, "protonSim", kRed + 1);
   auto *heliumG4 = MakeGraph(geant.heliumKinematics, "heliumG4", kBlue + 1);
   auto *heliumSim = MakeGraph(simple.heliumKinematics, "heliumSim", kRed + 1);
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
   auto *heliumMg = new TMultiGraph();
   heliumMg->SetTitle("He-4 kinematic locus;p_{T} [MeV/c];p_{Z} [MeV/c]");
   heliumMg->Add(heliumG4, "P");
   heliumMg->Add(heliumSim, "P");
   heliumMg->Draw("A");
   auto *heliumLegend = new TLegend(0.62, 0.78, 0.9, 0.9);
   heliumLegend->AddEntry(heliumG4, "Geant4", "p");
   heliumLegend->AddEntry(heliumSim, "SimpleSim", "p");
   heliumLegend->Draw();

   canvas->cd(3);
   geant.protonBragg->SetTitle("Bragg curve: proton");
   geant.protonBragg->Draw("hist");
   simple.protonBragg->Draw("hist same");
   auto *braggProtonLegend = new TLegend(0.62, 0.78, 0.9, 0.9);
   braggProtonLegend->AddEntry(geant.protonBragg, "Geant4", "l");
   braggProtonLegend->AddEntry(simple.protonBragg, "SimpleSim", "l");
   braggProtonLegend->Draw();

   canvas->cd(4);
   geant.heliumBragg->SetTitle("Bragg curve: He-4");
   geant.heliumBragg->Draw("hist");
   simple.heliumBragg->Draw("hist same");
   auto *braggHeliumLegend = new TLegend(0.62, 0.78, 0.9, 0.9);
   braggHeliumLegend->AddEntry(geant.heliumBragg, "Geant4", "l");
   braggHeliumLegend->AddEntry(simple.heliumBragg, "SimpleSim", "l");
   braggHeliumLegend->Draw();

   canvas->cd(5);
   geant.protonRange->SetTitle("Range distributions");
   geant.protonRange->Draw("hist");
   simple.protonRange->Draw("hist same");
   geant.heliumRange->Draw("hist same");
   simple.heliumRange->Draw("hist same");
   auto *rangeLegend = new TLegend(0.44, 0.64, 0.9, 0.9);
   rangeLegend->AddEntry(geant.protonRange, "Geant4 proton", "l");
   rangeLegend->AddEntry(simple.protonRange, "SimpleSim proton", "l");
   rangeLegend->AddEntry(geant.heliumRange, "Geant4 He-4", "l");
   rangeLegend->AddEntry(simple.heliumRange, "SimpleSim He-4", "l");
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
