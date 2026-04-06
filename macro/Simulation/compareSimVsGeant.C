/**
 * compareSimVsGeant.C
 *
 * Overlay AtTestSimulation (SimpleSim) and Geant4 simulation output on the
 * same plots for visual comparison.  Both produce "AtTpcPoint" branches
 * (TClonesArray of AtMCPoint / AtTpcPoint) in a "cbmsim" TTree.
 *
 * Comparison plots produced (saved to ./data/compareSimVsGeant.pdf):
 *   1. Bragg curve: mean dE/dx [MeV/mm] vs Z position [mm] — per-event tracks
 *      averaged across all events.
 *   2. Track-length distribution [mm].
 *   3. Total energy loss per track [MeV].
 *   4. XY hit projection (shows Larmor spirals when B ≠ 0).
 *   5. Z-position distribution of all hits.
 *
 * Usage:
 *   source build/config.sh
 *   root -l -q 'macro/Simulation/compareSimVsGeant.C("geant_output.root","simpleSim_output.root")'
 *
 * Arguments:
 *   geantFile    — output ROOT file from a standard FairRunSim Geant4 macro
 *   simpleFile   — output ROOT file from simpleSim_Bfield.C (or similar AtTestSimulation macro)
 *   branchName   — MCPoint branch name (default "AtTpcPoint"; Geant4 may use "AtTpcPoint")
 *   nEventsMax   — maximum events to read from each file (0 = all)
 */

#include <algorithm>
#include <iostream>
#include <map>
#include <string>

// ---------------------------------------------------------------------------
// Helper: fill histograms from one file
// ---------------------------------------------------------------------------
struct SimData {
   TH1D *hTrackLength;  // track length [mm]
   TH1D *hTotalELoss;   // total energy loss per track [MeV]
   TH1D *hHitZ;         // Z position of all hits [mm]
   TH2D *hXY;           // XY projection of all hits [mm]
   TProfile *hBragg;    // mean dE/dx [MeV/mm] vs Z [mm]
   int nEvents;
   int nHits;
};

SimData FillHistograms(const TString &fileName, const TString &branchName, int nEventsMax,
                       const TString &suffix)
{
   SimData d;
   d.hTrackLength = new TH1D("hLen_" + suffix, ";Track length [mm];Events", 200, 0, 600);
   d.hTotalELoss = new TH1D("hELoss_" + suffix, ";Total #DeltaE [MeV];Events", 200, 0, 100);
   d.hHitZ = new TH1D("hZ_" + suffix, ";Z [mm];Hits", 200, -100, 1100);
   d.hXY = new TH2D("hXY_" + suffix, ";X [mm];Y [mm]", 200, -300, 300, 200, -300, 300);
   d.hBragg = new TProfile("hBragg_" + suffix, ";Z [mm];#LTdE/dx#GT [MeV/mm]", 200, -100, 1100);
   d.nEvents = 0;
   d.nHits = 0;

   TFile *f = TFile::Open(fileName);
   if (!f || f->IsZombie()) {
      std::cerr << "ERROR: cannot open " << fileName << "\n";
      return d;
   }

   TTree *tree = dynamic_cast<TTree *>(f->Get("cbmsim"));
   if (!tree) {
      std::cerr << "ERROR: no 'cbmsim' tree in " << fileName << "\n";
      f->Close();
      return d;
   }

   TClonesArray *pointArray = nullptr;
   tree->SetBranchAddress(branchName, &pointArray);

   int nEvents = (nEventsMax > 0) ? std::min((int)tree->GetEntriesFast(), nEventsMax)
                                  : (int)tree->GetEntriesFast();

   for (int iEv = 0; iEv < nEvents; ++iEv) {
      tree->GetEntry(iEv);
      if (!pointArray)
         continue;

      // Group points by track ID, accumulate per-track quantities
      std::map<int, double> trackELoss;
      std::map<int, double> trackLength;

      int nPts = pointArray->GetEntriesFast();
      for (int i = 0; i < nPts; ++i) {
         auto *pt = dynamic_cast<FairMCPoint *>(pointArray->At(i));
         if (!pt)
            continue;

         double x_mm = pt->GetX() * 10.;
         double y_mm = pt->GetY() * 10.;
         double z_mm = pt->GetZ() * 10.;
         double eLoss_MeV = pt->GetEnergyLoss() * 1000.; // GeV → MeV
         double len_mm = pt->GetLength() * 10.;           // cm → mm

         d.hHitZ->Fill(z_mm);
         d.hXY->Fill(x_mm, y_mm);

         // dE/dx = energy loss / step length.  Use difference between consecutive
         // lengths on the same track as the step size.
         int tid = pt->GetTrackID();
         double prevLen = trackLength.count(tid) ? trackLength[tid] : 0;
         double stepLen = len_mm - prevLen;
         if (stepLen > 0)
            d.hBragg->Fill(z_mm, eLoss_MeV / stepLen);

         trackELoss[tid] += eLoss_MeV;
         trackLength[tid] = len_mm;
         d.nHits++;
      }

      for (auto &[tid, eLoss] : trackELoss) {
         d.hTotalELoss->Fill(eLoss);
         d.hTrackLength->Fill(trackLength[tid]);
      }
      d.nEvents++;
   }

   f->Close();
   std::cout << suffix << ": read " << d.nEvents << " events, " << d.nHits << " hits\n";
   return d;
}

// ---------------------------------------------------------------------------
// Main macro
// ---------------------------------------------------------------------------
void compareSimVsGeant(TString geantFile = "./data/geant_output.root",
                       TString simpleFile = "./data/simpleSim_Bfield.root",
                       TString branchName = "AtTpcPoint", int nEventsMax = 0)
{
   gStyle->SetOptStat(0);
   gStyle->SetOptTitle(0);

   SimData geant = FillHistograms(geantFile, branchName, nEventsMax, "G4");
   SimData simple = FillHistograms(simpleFile, branchName, nEventsMax, "Sim");

   if (geant.nHits == 0 && simple.nHits == 0) {
      std::cerr << "ERROR: no hits in either file. Check file paths and branch name.\n";
      return;
   }

   // ---- Style -----------------------------------------------------------
   auto styleG4 = [](TH1 *h) {
      h->SetLineColor(kBlue + 1);
      h->SetLineWidth(2);
   };
   auto styleSim = [](TH1 *h) {
      h->SetLineColor(kRed + 1);
      h->SetLineWidth(2);
      h->SetLineStyle(2);
   };

   styleG4(geant.hTrackLength);
   styleG4(geant.hTotalELoss);
   styleG4(geant.hHitZ);
   styleG4(geant.hBragg);
   styleSim(simple.hTrackLength);
   styleSim(simple.hTotalELoss);
   styleSim(simple.hHitZ);
   styleSim(simple.hBragg);

   // Normalize to events so shapes compare regardless of statistics
   auto normalize = [](TH1 *h, double n) {
      if (n > 0 && h->Integral() > 0)
         h->Scale(1.0 / h->Integral());
   };
   normalize(geant.hTrackLength, geant.nEvents);
   normalize(geant.hTotalELoss, geant.nEvents);
   normalize(geant.hHitZ, geant.nHits);
   normalize(simple.hTrackLength, simple.nEvents);
   normalize(simple.hTotalELoss, simple.nEvents);
   normalize(simple.hHitZ, simple.nHits);

   // ---- Canvas layout ---------------------------------------------------
   TCanvas *c = new TCanvas("cCompare", "Geant4 vs SimpleSim comparison", 1400, 900);
   c->Divide(3, 2);

   auto makeLegend = [&](TVirtualPad *pad) {
      pad->cd();
      auto *leg = new TLegend(0.55, 0.72, 0.92, 0.88);
      leg->SetBorderSize(0);
      leg->AddEntry(geant.hBragg, "Geant4", "l");
      leg->AddEntry(simple.hBragg, "SimpleSim", "l");
      leg->Draw();
   };

   // 1 — Bragg curve
   c->cd(1);
   gPad->SetLeftMargin(0.15);
   auto *braggTitle = new TH1D("braggFrame", ";Z [mm];#LTdE/dx#GT [MeV/mm]", 1, -100, 1100);
   braggTitle->SetMaximum(std::max(geant.hBragg->GetMaximum(), simple.hBragg->GetMaximum()) * 1.2);
   braggTitle->Draw();
   geant.hBragg->Draw("same");
   simple.hBragg->Draw("same");
   makeLegend(gPad);

   // 2 — Track length
   c->cd(2);
   gPad->SetLeftMargin(0.15);
   geant.hTrackLength->GetYaxis()->SetTitle("Normalised entries");
   geant.hTrackLength->SetMaximum(
      std::max(geant.hTrackLength->GetMaximum(), simple.hTrackLength->GetMaximum()) * 1.3);
   geant.hTrackLength->Draw("hist");
   simple.hTrackLength->Draw("hist same");
   makeLegend(gPad);

   // 3 — Total energy loss
   c->cd(3);
   gPad->SetLeftMargin(0.15);
   geant.hTotalELoss->GetYaxis()->SetTitle("Normalised entries");
   geant.hTotalELoss->SetMaximum(
      std::max(geant.hTotalELoss->GetMaximum(), simple.hTotalELoss->GetMaximum()) * 1.3);
   geant.hTotalELoss->Draw("hist");
   simple.hTotalELoss->Draw("hist same");
   makeLegend(gPad);

   // 4 — XY projection (Geant4)
   c->cd(4);
   gPad->SetLeftMargin(0.15);
   geant.hXY->GetZaxis()->SetTitle("Hits");
   geant.hXY->SetTitle("Geant4 XY hits");
   geant.hXY->Draw("colz");

   // 5 — XY projection (SimpleSim)
   c->cd(5);
   gPad->SetLeftMargin(0.15);
   simple.hXY->GetZaxis()->SetTitle("Hits");
   simple.hXY->SetTitle("SimpleSim XY hits");
   simple.hXY->Draw("colz");

   // 6 — Z hit distribution
   c->cd(6);
   gPad->SetLeftMargin(0.15);
   geant.hHitZ->GetYaxis()->SetTitle("Normalised entries");
   geant.hHitZ->SetMaximum(
      std::max(geant.hHitZ->GetMaximum(), simple.hHitZ->GetMaximum()) * 1.3);
   geant.hHitZ->Draw("hist");
   simple.hHitZ->Draw("hist same");
   makeLegend(gPad);

   // ---- Save ------------------------------------------------------------
   c->SaveAs("./data/compareSimVsGeant.pdf");
   std::cout << "Comparison plot saved to ./data/compareSimVsGeant.pdf\n";
}
