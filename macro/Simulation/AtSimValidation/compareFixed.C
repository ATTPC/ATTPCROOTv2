
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
   int trackID{};
   std::vector<PointSample> points;
};

struct EventTracks {
   TrackData proton;
   TrackData helium;
};

struct PlotData {
   std::vector<EventTracks> events;
   TProfile *protonBragg{nullptr};
   TProfile *heliumBragg{nullptr};
   TH1D *trackLength{nullptr};
   std::vector<std::pair<double, double>> xy;
   std::vector<std::pair<double, double>> xz;
};

bool SortByLength(const PointSample &lhs, const PointSample &rhs) { return lhs.length < rhs.length; }

TrackData BuildTrackData(int trackID, TClonesArray *points)
{
   TrackData out;
   out.trackID = trackID;
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

bool IsReactionEvent(const std::vector<int> &trackIDs) { return trackIDs.size() >= 2; }

PlotData LoadPlotData(const TString &fileName, const char *tag)
{
   PlotData out;
   out.protonBragg = new TProfile(Form("hProtonBragg_%s", tag), ";Z [mm];dE/dx [MeV/mm]", 100, 0., 1000.);
   out.heliumBragg = new TProfile(Form("hHeliumBragg_%s", tag), ";Z [mm];dE/dx [MeV/mm]", 100, 0., 1000.);
   out.trackLength = new TH1D(Form("hTrackLength_%s", tag), ";Track length [mm];Tracks", 120, 0., 500.);

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
      if (!IsReactionEvent(trackIDs))
         continue;

      EventTracks event;
      event.proton = BuildTrackData(trackIDs.front(), pointArray);
      event.helium = BuildTrackData(trackIDs.back(), pointArray);
      out.events.push_back(event);

      for (const auto *track : {&event.proton, &event.helium}) {
         if (track->points.empty())
            continue;

         out.trackLength->Fill(track->points.back().length);
         for (const auto &point : track->points) {
            out.xy.emplace_back(point.x, point.y);
            out.xz.emplace_back(point.z, point.x);
         }

         auto *profile = (track == &event.proton) ? out.protonBragg : out.heliumBragg;
         for (size_t i = 1; i < track->points.size(); ++i) {
            const auto &prev = track->points[i - 1];
            const auto &curr = track->points[i];
            double dx = curr.x - prev.x;
            double dy = curr.y - prev.y;
            double dz = curr.z - prev.z;
            double step = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (step > 1e-6 && curr.eLoss > 0.)
               profile->Fill(curr.z, curr.eLoss / step);
         }
      }
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

void StyleTrackLength(TH1D *hist, Color_t color, Style_t style)
{
   hist->SetLineColor(color);
   hist->SetLineWidth(2);
   hist->SetLineStyle(style);
}

TGraph *MakeProjectionGraph(const std::vector<std::pair<double, double>> &points, const char *name, Color_t color)
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

void DrawTrackOverlay(const PlotData &data, bool drawProton, Color_t color)
{
   int count = 0;
   for (const auto &event : data.events) {
      const auto &track = drawProton ? event.proton : event.helium;
      if (track.points.empty())
         continue;
      auto *line = new TPolyLine3D(track.points.size());
      line->SetLineColor(color);
      line->SetLineWidth(2);
      for (size_t i = 0; i < track.points.size(); ++i)
         line->SetPoint(i, track.points[i].x, track.points[i].y, track.points[i].z);
      if (count == 0) {
         line->Draw();
      } else {
         line->Draw("same");
      }
      if (++count >= 20)
         break;
   }
}
} // namespace

void compareFixed(TString geantFile = "./data/geant4_fixed.root", TString simpleFile = "./data/simpleSim_fixed.root")
{
   gStyle->SetOptStat(0);
   auto geant = LoadPlotData(geantFile, "g4");
   auto simple = LoadPlotData(simpleFile, "sim");

   StyleProfile(geant.protonBragg, kBlue + 1, 1);
   StyleProfile(simple.protonBragg, kRed + 1, 2);
   StyleProfile(geant.heliumBragg, kBlue + 1, 1);
   StyleProfile(simple.heliumBragg, kRed + 1, 2);
   StyleTrackLength(geant.trackLength, kBlue + 1, 1);
   StyleTrackLength(simple.trackLength, kRed + 1, 2);

   auto *xyG4 = MakeProjectionGraph(geant.xy, "xyG4", kBlue + 1);
   auto *xySim = MakeProjectionGraph(simple.xy, "xySim", kRed + 1);
   auto *xzG4 = MakeProjectionGraph(geant.xz, "xzG4", kBlue + 1);
   auto *xzSim = MakeProjectionGraph(simple.xz, "xzSim", kRed + 1);

   auto *canvas = new TCanvas("cFixedCompare", "AtSimpleSimulation fixed-angle validation", 1800, 1000);
   canvas->Divide(4, 2);

   canvas->cd(1);
   gPad->SetTheta(20);
   gPad->SetPhi(35);
   DrawTrackOverlay(geant, true, kBlue + 1);
   DrawTrackOverlay(simple, true, kRed + 1);
   {
      auto *legend = new TLegend(0.62, 0.78, 0.9, 0.9);
      legend->AddEntry((TObject *)nullptr, "Proton 3D overlay", "");
      legend->AddEntry((TObject *)nullptr, "Blue: Geant4", "");
      legend->AddEntry((TObject *)nullptr, "Red: SimpleSim", "");
      legend->Draw();
   }

   canvas->cd(2);
   gPad->SetTheta(20);
   gPad->SetPhi(35);
   DrawTrackOverlay(geant, false, kBlue + 1);
   DrawTrackOverlay(simple, false, kRed + 1);
   {
      auto *legend = new TLegend(0.62, 0.78, 0.9, 0.9);
      legend->AddEntry((TObject *)nullptr, "He-4 3D overlay", "");
      legend->AddEntry((TObject *)nullptr, "Blue: Geant4", "");
      legend->AddEntry((TObject *)nullptr, "Red: SimpleSim", "");
      legend->Draw();
   }

   canvas->cd(3);
   auto *xy = new TMultiGraph();
   xy->SetTitle("XY projection;X [mm];Y [mm]");
   xy->Add(xyG4, "P");
   xy->Add(xySim, "P");
   xy->Draw("A");
   auto *xyLegend = new TLegend(0.65, 0.78, 0.9, 0.9);
   xyLegend->AddEntry(xyG4, "Geant4", "p");
   xyLegend->AddEntry(xySim, "SimpleSim", "p");
   xyLegend->Draw();

   canvas->cd(4);
   auto *xz = new TMultiGraph();
   xz->SetTitle("XZ projection;Z [mm];X [mm]");
   xz->Add(xzG4, "P");
   xz->Add(xzSim, "P");
   xz->Draw("A");
   auto *xzLegend = new TLegend(0.65, 0.78, 0.9, 0.9);
   xzLegend->AddEntry(xzG4, "Geant4", "p");
   xzLegend->AddEntry(xzSim, "SimpleSim", "p");
   xzLegend->Draw();

   canvas->cd(5);
   geant.protonBragg->SetTitle("Bragg curve: proton");
   geant.protonBragg->Draw("hist");
   simple.protonBragg->Draw("hist same");
   auto *protonLegend = new TLegend(0.6, 0.78, 0.9, 0.9);
   protonLegend->AddEntry(geant.protonBragg, "Geant4", "l");
   protonLegend->AddEntry(simple.protonBragg, "SimpleSim", "l");
   protonLegend->Draw();

   canvas->cd(6);
   geant.heliumBragg->SetTitle("Bragg curve: He-4");
   geant.heliumBragg->Draw("hist");
   simple.heliumBragg->Draw("hist same");
   auto *heliumLegend = new TLegend(0.6, 0.78, 0.9, 0.9);
   heliumLegend->AddEntry(geant.heliumBragg, "Geant4 Bragg", "l");
   heliumLegend->AddEntry(simple.heliumBragg, "SimpleSim Bragg", "l");
   heliumLegend->Draw();

   canvas->cd(7);
   geant.trackLength->SetTitle("Track length distribution");
   geant.trackLength->Draw("hist");
   simple.trackLength->Draw("hist same");
   auto *trackLegend = new TLegend(0.6, 0.78, 0.9, 0.9);
   trackLegend->AddEntry(geant.trackLength, "Geant4", "l");
   trackLegend->AddEntry(simple.trackLength, "SimpleSim", "l");
   trackLegend->Draw();

   gSystem->mkdir("data", kTRUE);
   canvas->SaveAs("./data/compareFixed.pdf");
}
