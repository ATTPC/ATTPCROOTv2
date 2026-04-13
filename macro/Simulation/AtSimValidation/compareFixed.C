#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <AtMCTrack.h>
#include <FairMCPoint.h>
#include <TCanvas.h>
#include <TClonesArray.h>
#include <TFile.h>
#include <TGraph.h>
#include <TLegend.h>
#include <TLine.h>
#include <TMath.h>
#include <TPaveText.h>
#include <TPolyMarker3D.h>
#include <TPolyLine3D.h>
#include <TStyle.h>
#include <TTree.h>

namespace {
constexpr double kProtonMassMeV = 938.27208816;

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

struct ReactionEvent {
   int fairEvent{-1};
   int reactionIndex{-1};
   double truthKE{0.0};
   double truthTheta{0.0};
   double truthPhi{0.0};
   TrackData proton;
};

struct MatchSummary {
   std::vector<std::pair<ReactionEvent, ReactionEvent>> usablePairs;
   size_t geantOnly{0};
   size_t simpleOnly{0};
   size_t geantIncomplete{0};
   size_t simpleIncomplete{0};
};

bool SortByLength(const PointSample &lhs, const PointSample &rhs) { return lhs.length < rhs.length; }

void ExpandRange(double value, double &minValue, double &maxValue)
{
   minValue = std::min(minValue, value);
   maxValue = std::max(maxValue, value);
}

std::pair<double, double> AddMargin(double minValue, double maxValue)
{
   const double span = std::max(1.0, maxValue - minValue);
   return {minValue - 0.08 * span, maxValue + 0.08 * span};
}

double GetMaxStoppingPower(const TrackData &track)
{
   double maxValue = 0.;
   for (size_t i = 1; i < track.points.size(); ++i) {
      const auto &prev = track.points[i - 1];
      const auto &curr = track.points[i];
      const double dx = curr.x - prev.x;
      const double dy = curr.y - prev.y;
      const double dz = curr.z - prev.z;
      const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (step <= 1e-6 || curr.eLoss <= 0.)
         continue;
      maxValue = std::max(maxValue, curr.eLoss / step);
   }
   return maxValue;
}

std::pair<double, double> GetCoordinateRange(const TrackData &lhs, const TrackData &rhs, char axis)
{
   double minValue = std::numeric_limits<double>::max();
   double maxValue = std::numeric_limits<double>::lowest();
   auto updateTrack = [&](const TrackData &track) {
      for (const auto &point : track.points) {
         double value = 0.;
         switch (axis) {
         case 'x':
            value = point.x;
            break;
         case 'y':
            value = point.y;
            break;
         case 'z':
            value = point.z;
            break;
         default:
            return;
         }
         ExpandRange(value, minValue, maxValue);
      }
   };
   updateTrack(lhs);
   updateTrack(rhs);
   return AddMargin(minValue, maxValue);
}

double GetPathLength(const TrackData &track)
{
   if (track.points.size() < 2)
      return 0.;
   return track.points.back().length - track.points.front().length;
}

double GetFinalZ(const TrackData &track)
{
   if (track.points.empty())
      return 0.;
   return track.points.back().z;
}

double GetInitialThetaDeg(const TrackData &track)
{
   if (track.points.empty())
      return 0.;
   const auto &point = track.points.front();
   return std::atan2(std::hypot(point.px, point.py), point.pz) * TMath::RadToDeg();
}

double GetInitialPhiDeg(const TrackData &track)
{
   if (track.points.empty())
      return 0.;
   const auto &point = track.points.front();
   return std::atan2(point.py, point.px) * TMath::RadToDeg();
}

double GetInitialKineticEnergyMeV(const TrackData &track)
{
   if (track.points.empty())
      return 0.;
   const auto &point = track.points.front();
   const double momentum = std::sqrt(point.px * point.px + point.py * point.py + point.pz * point.pz);
   return std::sqrt(momentum * momentum + kProtonMassMeV * kProtonMassMeV) - kProtonMassMeV;
}

double GetTrackThetaDeg(const AtMCTrack &track)
{
   return std::atan2(std::hypot(track.GetPx(), track.GetPy()), track.GetPz()) * TMath::RadToDeg();
}

double GetTrackPhiDeg(const AtMCTrack &track) { return std::atan2(track.GetPy(), track.GetPx()) * TMath::RadToDeg(); }

double GetTrackKineticEnergyMeV(const AtMCTrack &track)
{
   const double px = track.GetPx() * 1000.0;
   const double py = track.GetPy() * 1000.0;
   const double pz = track.GetPz() * 1000.0;
   const double momentum = std::sqrt(px * px + py * py + pz * pz);
   return std::sqrt(momentum * momentum + kProtonMassMeV * kProtonMassMeV) - kProtonMassMeV;
}

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

bool IsPrimaryProton(AtMCTrack *track)
{
   return track != nullptr && track->GetMotherId() == -1 && track->GetPdgCode() == 2212;
}

std::vector<ReactionEvent> LoadReactionEvents(const TString &fileName)
{
   std::vector<ReactionEvent> events;

   auto *file = TFile::Open(fileName);
   if (!file || file->IsZombie()) {
      std::cerr << "Cannot open " << fileName << "\n";
      return events;
   }

   auto *tree = dynamic_cast<TTree *>(file->Get("cbmsim"));
   if (!tree) {
      std::cerr << "Missing cbmsim tree in " << fileName << "\n";
      file->Close();
      return events;
   }

   TClonesArray *pointArray = nullptr;
   TClonesArray *trackArray = nullptr;
   tree->SetBranchAddress("AtTpcPoint", &pointArray);
   tree->SetBranchAddress("MCTrack", &trackArray);

   for (Long64_t iEvent = 1; iEvent < tree->GetEntries(); iEvent += 2) {
      tree->GetEntry(iEvent);
      if (!trackArray)
         continue;

      int protonTrackID = -1;
      for (int i = 0; i < trackArray->GetEntriesFast(); ++i) {
         auto *track = dynamic_cast<AtMCTrack *>(trackArray->At(i));
         if (IsPrimaryProton(track)) {
            protonTrackID = i;
            break;
         }
      }

      if (protonTrackID < 0)
         continue;

      auto *protonTrack = dynamic_cast<AtMCTrack *>(trackArray->At(protonTrackID));
      if (protonTrack == nullptr)
         continue;

      auto proton = BuildTrackData(protonTrackID, pointArray);
      events.push_back({static_cast<int>(iEvent), static_cast<int>(iEvent / 2), GetTrackKineticEnergyMeV(*protonTrack),
                        GetTrackThetaDeg(*protonTrack), GetTrackPhiDeg(*protonTrack), std::move(proton)});
   }

   file->Close();
   return events;
}

MatchSummary MatchReactionEvents(const std::vector<ReactionEvent> &geant, const std::vector<ReactionEvent> &simple)
{
   MatchSummary summary;

   std::map<int, const ReactionEvent *> simpleByTruth;
   for (const auto &event : simple)
      simpleByTruth[event.reactionIndex] = &event;

   for (const auto &event : geant) {
      auto it = simpleByTruth.find(event.reactionIndex);
      if (it == simpleByTruth.end()) {
         ++summary.geantOnly;
         continue;
      }

      const auto &simpleEvent = *it->second;
      simpleByTruth.erase(it);

      const bool geantUsable = event.proton.points.size() >= 2;
      const bool simpleUsable = simpleEvent.proton.points.size() >= 2;
      if (!geantUsable)
         ++summary.geantIncomplete;
      if (!simpleUsable)
         ++summary.simpleIncomplete;
      if (!geantUsable || !simpleUsable)
         continue;

      summary.usablePairs.emplace_back(event, simpleEvent);
   }

   summary.simpleOnly = simpleByTruth.size();
   return summary;
}

TGraph *MakeProjectionGraph(const TrackData &track, bool xz, const char *name, Color_t color, Style_t style)
{
   auto *graph = new TGraph(track.points.size());
   graph->SetName(name);
   graph->SetLineColorAlpha(color, 0.65);
   graph->SetLineWidth(3);
   graph->SetLineStyle(style);
   graph->SetMarkerColorAlpha(color, 0.55);
   graph->SetMarkerStyle(style == 1 ? 20 : 24);
   graph->SetMarkerSize(0.55);
   for (size_t i = 0; i < track.points.size(); ++i) {
      const auto &point = track.points[i];
      graph->SetPoint(i, xz ? point.z : point.x, xz ? point.x : point.y);
   }
   return graph;
}

TPolyLine3D *MakeTrackLine3D(const TrackData &track, const char *name, Color_t color, Style_t style)
{
   auto *line = new TPolyLine3D(track.points.size());
   (void)name;
   line->SetLineColorAlpha(color, 0.55);
   line->SetLineWidth(4);
   line->SetLineStyle(style);
   for (size_t i = 0; i < track.points.size(); ++i)
      line->SetPoint(i, track.points[i].x, track.points[i].y, track.points[i].z);
   return line;
}

TPolyMarker3D *MakeTrackMarkers3D(const TrackData &track, Color_t color, Style_t style)
{
   const int stride = std::max<size_t>(1, track.points.size() / 60);
   const int nMarkers = static_cast<int>((track.points.size() + stride - 1) / stride);
   auto *markers = new TPolyMarker3D(nMarkers);
   markers->SetMarkerColorAlpha(color, 0.75);
   markers->SetMarkerStyle(style == 1 ? 20 : 24);
   markers->SetMarkerSize(0.6);

   int index = 0;
   for (size_t i = 0; i < track.points.size(); i += stride)
      markers->SetPoint(index++, track.points[i].x, track.points[i].y, track.points[i].z);
   return markers;
}

TGraph *MakeResidualRangeGraph(const TrackData &track, const char *name, Color_t color, Style_t style)
{
   std::vector<std::pair<double, double>> samples;
   const double startLength = track.points.front().length;
   const double totalPath = GetPathLength(track);

   for (size_t i = 1; i < track.points.size(); ++i) {
      const auto &prev = track.points[i - 1];
      const auto &curr = track.points[i];
      const double dx = curr.x - prev.x;
      const double dy = curr.y - prev.y;
      const double dz = curr.z - prev.z;
      const double step = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (step <= 1e-6 || curr.eLoss <= 0.)
         continue;

      const double travelled = curr.length - startLength;
      const double residualRange = std::max(0.0, totalPath - travelled);
      samples.emplace_back(residualRange, curr.eLoss / step);
   }

   auto *graph = new TGraph(samples.size());
   graph->SetName(name);
   graph->SetLineColor(color);
   graph->SetLineWidth(2);
   graph->SetLineStyle(style);
   graph->SetMarkerColor(color);
   graph->SetMarkerStyle(20);
   graph->SetMarkerSize(0.35);
   for (size_t i = 0; i < samples.size(); ++i)
      graph->SetPoint(i, samples[i].first, samples[i].second);
   return graph;
}

TGraph *MakeScatterGraph(const std::vector<std::pair<double, double>> &pairs, const char *name, Color_t color)
{
   auto *graph = new TGraph(pairs.size());
   graph->SetName(name);
   graph->SetMarkerStyle(20);
   graph->SetMarkerSize(0.8);
   graph->SetMarkerColor(color);
   graph->SetLineColor(color);
   for (size_t i = 0; i < pairs.size(); ++i)
      graph->SetPoint(i, pairs[i].first, pairs[i].second);
   return graph;
}

void DrawIdentityScatter(const std::vector<std::pair<double, double>> &pairs, const char *graphName, Color_t color,
                         const TString &title, const TString &xTitle, const TString &yTitle)
{
   if (pairs.empty()) {
      auto *box = new TPaveText(0.2, 0.4, 0.8, 0.6, "NDC");
      box->AddText("No matched proton events");
      box->Draw();
      return;
   }

   double minValue = std::numeric_limits<double>::max();
   double maxValue = std::numeric_limits<double>::lowest();
   for (const auto &[x, y] : pairs) {
      minValue = std::min(minValue, std::min(x, y));
      maxValue = std::max(maxValue, std::max(x, y));
   }
   const double span = std::max(1.0, maxValue - minValue);
   minValue -= 0.08 * span;
   maxValue += 0.08 * span;

   auto *frame = gPad->DrawFrame(minValue, minValue, maxValue, maxValue, title);
   frame->GetXaxis()->SetTitle(xTitle);
   frame->GetYaxis()->SetTitle(yTitle);
   frame->GetYaxis()->SetTitleOffset(1.3);

   auto *graph = MakeScatterGraph(pairs, graphName, color);
   graph->Draw("P");

   auto *diag = new TLine(minValue, minValue, maxValue, maxValue);
   diag->SetLineStyle(2);
   diag->SetLineColor(kGray + 2);
   diag->Draw();
}

double PhiDiffDeg(double lhs, double rhs)
{
   double diff = lhs - rhs;
   while (diff > 180.0)
      diff -= 360.0;
   while (diff < -180.0)
      diff += 360.0;
   return diff;
}
} // namespace

void compareFixed(TString geantFile = "./data/geant4_fixed.root", TString simpleFile = "./data/simpleSim_fixed.root",
                  Int_t reactionIndex = 0)
{
   gStyle->SetOptStat(0);

   const auto geant = LoadReactionEvents(geantFile);
   const auto simple = LoadReactionEvents(simpleFile);
   const auto matchSummary = MatchReactionEvents(geant, simple);
   const auto &matched = matchSummary.usablePairs;
   const size_t matchedPairs = matched.size();
   if (matchedPairs == 0) {
      std::cerr << "No truth-matched proton reaction events with >=2 active-volume points found.\n";
      return;
   }

   const size_t selectedPair = std::clamp<int>(reactionIndex, 0, static_cast<int>(matchedPairs - 1));
   const auto &geantEvent = matched[selectedPair].first;
   const auto &simpleEvent = matched[selectedPair].second;

   std::vector<std::pair<double, double>> kineticPairs;
   std::vector<std::pair<double, double>> thetaPairs;
   std::vector<std::pair<double, double>> phiPairs;
   std::vector<std::pair<double, double>> pathPairs;
   std::vector<std::pair<double, double>> finalZPairs;
   kineticPairs.reserve(matchedPairs);
   thetaPairs.reserve(matchedPairs);
   phiPairs.reserve(matchedPairs);
   pathPairs.reserve(matchedPairs);
   finalZPairs.reserve(matchedPairs);
   size_t kinematicAgreementPairs = 0;

   for (size_t i = 0; i < matchedPairs; ++i) {
      const auto &geantPair = matched[i].first;
      const auto &simplePair = matched[i].second;
      const double geantKE = GetInitialKineticEnergyMeV(geantPair.proton);
      const double simpleKE = GetInitialKineticEnergyMeV(simplePair.proton);
      const double geantTheta = GetInitialThetaDeg(geantPair.proton);
      const double simpleTheta = GetInitialThetaDeg(simplePair.proton);
      const double geantPhi = GetInitialPhiDeg(geantPair.proton);
      const double simplePhi = GetInitialPhiDeg(simplePair.proton);
      const double geantPath = GetPathLength(geantPair.proton);
      const double simplePath = GetPathLength(simplePair.proton);

      kineticPairs.emplace_back(geantKE, simpleKE);
      thetaPairs.emplace_back(geantTheta, simpleTheta);
      phiPairs.emplace_back(geantPhi, simplePhi);
      pathPairs.emplace_back(geantPath, simplePath);
      finalZPairs.emplace_back(GetFinalZ(geantPair.proton), GetFinalZ(simplePair.proton));
      if (std::abs(geantKE - simpleKE) < 1.0 && std::abs(geantTheta - simpleTheta) < 0.2 &&
          std::abs(PhiDiffDeg(geantPhi, simplePhi)) < 0.5)
         ++kinematicAgreementPairs;
   }

   auto *track3DG4 = MakeTrackLine3D(geantEvent.proton, "gFixed3DG4", kBlue + 1, 1);
   auto *track3DSim = MakeTrackLine3D(simpleEvent.proton, "gFixed3DSim", kRed + 1, 2);
   auto *mark3DG4 = MakeTrackMarkers3D(geantEvent.proton, kBlue + 1, 1);
   auto *mark3DSim = MakeTrackMarkers3D(simpleEvent.proton, kRed + 1, 2);
   auto *xzG4 = MakeProjectionGraph(geantEvent.proton, true, "gFixedXZG4", kBlue + 1, 1);
   auto *xzSim = MakeProjectionGraph(simpleEvent.proton, true, "gFixedXZSim", kRed + 1, 2);
   auto *xyG4 = MakeProjectionGraph(geantEvent.proton, false, "gFixedXYG4", kBlue + 1, 1);
   auto *xySim = MakeProjectionGraph(simpleEvent.proton, false, "gFixedXYSim", kRed + 1, 2);
   auto *braggG4 = MakeResidualRangeGraph(geantEvent.proton, "gFixedBraggG4", kBlue + 1, 1);
   auto *braggSim = MakeResidualRangeGraph(simpleEvent.proton, "gFixedBraggSim", kRed + 1, 2);

   auto *canvas = new TCanvas("cFixedCompare", "AtSimTransport fixed-angle proton comparison", 1900, 1000);
   canvas->Divide(4, 2);

   canvas->cd(1);
   gPad->SetTheta(22);
   gPad->SetPhi(32);
   track3DG4->Draw();
   track3DSim->Draw("same");
   mark3DG4->Draw();
   mark3DSim->Draw();
   {
      auto *legend = new TLegend(0.56, 0.74, 0.9, 0.9);
      legend->AddEntry(track3DG4, Form("Geant4 event %d", geantEvent.fairEvent), "lp");
      legend->AddEntry(track3DSim, Form("SimpleSim event %d", simpleEvent.fairEvent), "lp");
      legend->AddEntry((TObject *)nullptr, Form("Reaction %d proton 3D", geantEvent.reactionIndex), "");
      legend->Draw();
   }

   canvas->cd(2);
   const auto [zMin, zMax] = GetCoordinateRange(geantEvent.proton, simpleEvent.proton, 'z');
   const auto [xMin, xMax] = GetCoordinateRange(geantEvent.proton, simpleEvent.proton, 'x');
   auto *xzFrame =
      gPad->DrawFrame(zMin, xMin, zMax, xMax, Form("Matched proton XZ track: reaction %d;Z [mm];X [mm]",
                                                   geantEvent.reactionIndex));
   xzFrame->GetYaxis()->SetTitleOffset(1.2);
   xzG4->Draw("LP");
   xzSim->Draw("LP");
   {
      auto *legend = new TLegend(0.58, 0.74, 0.9, 0.9);
      legend->AddEntry(xzG4, Form("Geant4 event %d", geantEvent.fairEvent), "lp");
      legend->AddEntry(xzSim, Form("SimpleSim event %d", simpleEvent.fairEvent), "lp");
      legend->Draw();
   }

   canvas->cd(3);
   const auto [xyXMin, xyXMax] = GetCoordinateRange(geantEvent.proton, simpleEvent.proton, 'x');
   const auto [xyYMin, xyYMax] = GetCoordinateRange(geantEvent.proton, simpleEvent.proton, 'y');
   auto *xyFrame =
      gPad->DrawFrame(xyXMin, xyYMin, xyXMax, xyYMax, Form("Matched proton XY track: reaction %d;X [mm];Y [mm]",
                                                           geantEvent.reactionIndex));
   xyFrame->GetYaxis()->SetTitleOffset(1.2);
   xyG4->Draw("LP");
   xySim->Draw("LP");
   {
      auto *legend = new TLegend(0.58, 0.74, 0.9, 0.9);
      legend->AddEntry(xyG4, "Geant4 proton", "lp");
      legend->AddEntry(xySim, "SimpleSim proton", "lp");
      legend->Draw();
   }

   canvas->cd(4);
   const double maxResidualRange = std::max(GetPathLength(geantEvent.proton), GetPathLength(simpleEvent.proton));
   const double maxStoppingPower =
      1.15 * std::max(GetMaxStoppingPower(geantEvent.proton), GetMaxStoppingPower(simpleEvent.proton));
   auto *braggFrame = gPad->DrawFrame(0., 0., std::max(1.0, maxResidualRange), std::max(0.01, maxStoppingPower),
                                      "Matched proton stopping power;Residual range [mm];dE/ds [MeV/mm]");
   braggFrame->GetYaxis()->SetTitleOffset(1.25);
   braggG4->Draw("LP");
   braggSim->Draw("LP");
   {
      auto *legend = new TLegend(0.55, 0.74, 0.9, 0.9);
      legend->AddEntry(braggG4, "Geant4 proton", "lp");
      legend->AddEntry(braggSim, "SimpleSim proton", "lp");
      legend->Draw();
   }

   canvas->cd(5);
   DrawIdentityScatter(kineticPairs, "gFixedEnergyScatter", kBlue + 1, "Matched proton kinetic energy; ; ",
                       "Geant4 proton KE [MeV]", "SimpleSim proton KE [MeV]");

   canvas->cd(6);
   DrawIdentityScatter(thetaPairs, "gFixedThetaScatter", kBlue + 1, "Matched proton lab angle; ; ",
                       "Geant4 proton #theta_{lab} [deg]", "SimpleSim proton #theta_{lab} [deg]");

   canvas->cd(7);
   DrawIdentityScatter(phiPairs, "gFixedPhiScatter", kBlue + 1, "Matched proton lab azimuth; ; ",
                       "Geant4 proton #phi_{lab} [deg]", "SimpleSim proton #phi_{lab} [deg]");

   canvas->cd(8);
   DrawIdentityScatter(pathPairs, "gFixedPathScatter", kBlue + 1, "Matched proton path length; ; ",
                       "Geant4 proton path length [mm]", "SimpleSim proton path length [mm]");
   auto *note = new TPaveText(0.14, 0.72, 0.52, 0.9, "NDC");
   note->SetFillStyle(0);
   note->SetBorderSize(1);
   note->AddText(Form("Truth-matched usable pairs: %zu", matchedPairs));
   note->AddText(Form("Generator-only Geant/Simple: %zu / %zu", matchSummary.geantOnly, matchSummary.simpleOnly));
   note->AddText(
      Form("Incomplete Geant/Simple: %zu / %zu", matchSummary.geantIncomplete, matchSummary.simpleIncomplete));
   note->AddText(Form("Initial-state agreement pairs: %zu", kinematicAgreementPairs));
   note->AddText(Form("Selected reaction: %d", geantEvent.reactionIndex));
   note->AddText(Form("#Delta#phi selected: %.3f deg",
                      PhiDiffDeg(GetInitialPhiDeg(geantEvent.proton), GetInitialPhiDeg(simpleEvent.proton))));
   note->AddText(Form("Final Z pair: %.2f mm vs %.2f mm", finalZPairs[selectedPair].first, finalZPairs[selectedPair].second));
   note->Draw();

   std::cout << "compareFixed: usable truth-matched pairs " << matchedPairs << ", generator-only Geant/Simple "
             << matchSummary.geantOnly << "/" << matchSummary.simpleOnly << ", incomplete Geant/Simple "
             << matchSummary.geantIncomplete << "/" << matchSummary.simpleIncomplete << ", selected reaction "
             << geantEvent.reactionIndex << " (Geant4 fair event " << geantEvent.fairEvent << ", SimpleSim fair event "
             << simpleEvent.fairEvent << ").\n";

   gSystem->mkdir("data", kTRUE);
   canvas->SaveAs("./data/compareFixed.pdf");
}
