/**
 * compareSimVsGeant.C
 *
 * Overlay AtTestSimulation (SimpleSim) and Geant4 simulation output on the
 * same plots for visual comparison.  Both produce "AtTpcPoint" branches
 * (TClonesArray of AtMCPoint / FairMCPoint subclass) in a "cbmsim" TTree.
 *
 * Comparison plots produced (saved to ./data/compareSimVsGeant.pdf):
 *   1. Bragg curve: mean dE/dx [MeV/mm] vs Z position [mm].
 *   2. Track-length distribution [mm].
 *   3. Total energy loss per track [MeV].
 *   4. XY hit projection.
 *   5. Z-position distribution of all hits.
 *
 * dE/dx is computed as eLoss_step / step_length, where step_length is the
 * 3-D distance between consecutive hits on the same track.  This avoids the
 * bias that arises when using GetLength() (cumulative from track origin),
 * which for Geant4 includes path outside the active volume.
 *
 * Usage:
 *   source build/config.sh
 *   root -l -q 'macro/Simulation/compareSimVsGeant.C("geant.root","simple.root")'
 *
 * Arguments:
 *   geantFile   — output ROOT file from a FairRunSim Geant4 macro
 *   simpleFile  — output ROOT file from simpleSim_Bfield.C
 *   branchName  — MCPoint branch name (default "AtTpcPoint")
 *   nEventsMax  — max events to read per file (0 = all)
 */

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <string>

// ---------------------------------------------------------------------------
struct SimData {
   TH1D *hTrackLength;
   TH1D *hTotalELoss;
   TH1D *hHitZ;
   TH2D *hXY;
   TProfile *hBragg;
   int nEvents{0};
   int nHits{0};
};

SimData FillHistograms(const TString &fileName, const TString &branchName, int nEventsMax,
                       const TString &suffix)
{
   SimData d;
   d.hTrackLength = new TH1D("hLen_" + suffix, ";Track length [mm];Entries", 150, 0, 1100);
   d.hTotalELoss  = new TH1D("hELoss_" + suffix, ";Total #DeltaE [MeV];Entries", 150, 0, 15);
   d.hHitZ        = new TH1D("hZ_" + suffix, ";Z [mm];Hits", 100, 0, 1100);
   d.hXY          = new TH2D("hXY_" + suffix, ";X [mm];Y [mm]", 100, -300, 300, 100, -300, 300);
   d.hBragg       = new TProfile("hBragg_" + suffix, ";Z [mm];dE/dx [MeV/mm]", 100, 0, 1100);

   TFile *f = TFile::Open(fileName);
   if (!f || f->IsZombie()) {
      std::cerr << "WARNING: cannot open " << fileName << " — skipping.\n";
      return d;
   }

   TTree *tree = dynamic_cast<TTree *>(f->Get("cbmsim"));
   if (!tree) {
      std::cerr << "WARNING: no 'cbmsim' tree in " << fileName << " — skipping.\n";
      f->Close();
      return d;
   }

   TClonesArray *pointArray = nullptr;
   tree->SetBranchAddress(branchName, &pointArray);

   int nEvents = (nEventsMax > 0 && nEventsMax < (int)tree->GetEntriesFast())
                    ? nEventsMax
                    : (int)tree->GetEntriesFast();

   for (int iEv = 0; iEv < nEvents; ++iEv) {
      tree->GetEntry(iEv);
      if (!pointArray || pointArray->GetEntriesFast() == 0)
         continue;

      // Per-track accumulators (reset each event)
      std::map<int, double> trackELoss;
      std::map<int, double> trackLastLen;

      // Previous hit position per track (for step-length calculation)
      std::map<int, double> prevX, prevY, prevZ;

      int nPts = pointArray->GetEntriesFast();
      for (int i = 0; i < nPts; ++i) {
         auto *pt = dynamic_cast<FairMCPoint *>(pointArray->At(i));
         if (!pt)
            continue;

         double x_mm    = pt->GetX() * 10.;              // cm → mm
         double y_mm    = pt->GetY() * 10.;
         double z_mm    = pt->GetZ() * 10.;
         double eLoss   = pt->GetEnergyLoss() * 1000.;   // GeV → MeV
         double len_mm  = pt->GetLength() * 10.;          // cm → mm
         int tid        = pt->GetTrackID();

         d.hHitZ->Fill(z_mm);
         d.hXY->Fill(x_mm, y_mm);

         // --- Bragg curve: dE/dx from 3-D step between consecutive hits ----
         // Using position differences avoids the bias from GetLength() which
         // counts path outside the active volume for Geant4 tracks.
         if (prevZ.count(tid)) {
            double dx   = x_mm - prevX[tid];
            double dy   = y_mm - prevY[tid];
            double dz   = z_mm - prevZ[tid];
            double step = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (step > 0.01 && eLoss > 0)   // guard: skip zero-length or zero-loss steps
               d.hBragg->Fill(z_mm, eLoss / step);
         }
         prevX[tid] = x_mm;
         prevY[tid] = y_mm;
         prevZ[tid] = z_mm;

         trackELoss[tid]   += eLoss;
         trackLastLen[tid]  = len_mm;
         d.nHits++;
      }

      // Per-track summary histograms
      for (auto &[tid, eLoss] : trackELoss) {
         d.hTotalELoss->Fill(eLoss);
         d.hTrackLength->Fill(trackLastLen[tid]);
      }
      d.nEvents++;
   }

   f->Close();
   std::cout << suffix << ": " << d.nEvents << " events, " << d.nHits << " hits\n";
   return d;
}

// ---------------------------------------------------------------------------
void compareSimVsGeant(TString geantFile  = "./data/geant_output.root",
                       TString simpleFile = "./data/simpleSim_Bfield.root",
                       TString branchName = "AtTpcPoint",
                       int     nEventsMax = 0)
{
   gStyle->SetOptStat(0);
   gStyle->SetOptTitle(1);

   SimData geant  = FillHistograms(geantFile,  branchName, nEventsMax, "G4");
   SimData simple = FillHistograms(simpleFile, branchName, nEventsMax, "Sim");

   if (geant.nHits == 0 && simple.nHits == 0) {
      std::cerr << "ERROR: no hits in either file. Check paths and branch name.\n";
      return;
   }

   // ---- Style -----------------------------------------------------------
   auto styleG4 = [](TH1 *h, bool fill = false) {
      h->SetLineColor(kBlue + 1);
      h->SetLineWidth(2);
      if (fill) { h->SetFillColorAlpha(kBlue + 1, 0.2); h->SetFillStyle(1001); }
   };
   auto styleSim = [](TH1 *h, bool fill = false) {
      h->SetLineColor(kRed + 1);
      h->SetLineWidth(2);
      h->SetLineStyle(2);
      if (fill) { h->SetFillColorAlpha(kRed + 1, 0.2); h->SetFillStyle(1001); }
   };

   styleG4(geant.hTrackLength);   styleG4(geant.hTotalELoss);
   styleG4(geant.hHitZ);          styleG4(geant.hBragg);
   styleSim(simple.hTrackLength); styleSim(simple.hTotalELoss);
   styleSim(simple.hHitZ);        styleSim(simple.hBragg);

   // Normalize 1-D histograms to unit area so shapes compare
   // regardless of event count.  Skip empty histograms.
   auto normalize = [](TH1 *h) {
      if (h->Integral() > 0) h->Scale(1.0 / h->Integral());
   };
   if (geant.nEvents  > 0) { normalize(geant.hTrackLength);  normalize(geant.hTotalELoss);  normalize(geant.hHitZ); }
   if (simple.nEvents > 0) { normalize(simple.hTrackLength); normalize(simple.hTotalELoss); normalize(simple.hHitZ); }

   // ---- Canvas ----------------------------------------------------------
   TCanvas *c = new TCanvas("cCompare", "Geant4 vs SimpleSim", 1400, 900);
   c->Divide(3, 2);

   auto addLegend = [&](TVirtualPad *pad) {
      pad->cd();
      auto *leg = new TLegend(0.55, 0.72, 0.92, 0.88);
      leg->SetBorderSize(0);
      if (geant.nHits  > 0) leg->AddEntry(geant.hBragg,  "Geant4",   "l");
      if (simple.nHits > 0) leg->AddEntry(simple.hBragg, "SimpleSim","l");
      leg->Draw();
   };

   auto drawPair = [](TVirtualPad *p, TH1 *hG4, TH1 *hSim, bool profile = false) {
      p->cd(); p->SetLeftMargin(0.15);
      bool haveG4  = hG4  && hG4->GetEntries()  > 0;
      bool haveSim = hSim && hSim->GetEntries() > 0;
      double ymax  = 0;
      if (haveG4)  ymax = std::max(ymax, hG4->GetMaximum());
      if (haveSim) ymax = std::max(ymax, hSim->GetMaximum());
      if (ymax == 0) ymax = 1;
      TH1 *first = haveG4 ? hG4 : hSim;
      if (!first) return;
      first->SetMaximum(ymax * 1.25);
      first->Draw(profile ? "hist" : "hist");
      if (haveG4  && hG4  != first) hG4->Draw("hist same");
      if (haveSim && hSim != first) hSim->Draw("hist same");
   };

   // 1 — Bragg curve
   c->cd(1); gPad->SetLeftMargin(0.15);
   {
      bool haveG4  = geant.hBragg->GetEntries()  > 0;
      bool haveSim = simple.hBragg->GetEntries() > 0;
      double ymax = std::max(haveG4  ? geant.hBragg->GetMaximum()  : 0.,
                             haveSim ? simple.hBragg->GetMaximum() : 0.);
      if (ymax == 0) ymax = 1;
      TH1 *first = haveG4 ? (TH1*)geant.hBragg : (TH1*)simple.hBragg;
      first->SetMaximum(ymax * 1.25);
      first->Draw();
      if (haveG4  && (TH1*)geant.hBragg  != first) geant.hBragg->Draw("same");
      if (haveSim && (TH1*)simple.hBragg != first) simple.hBragg->Draw("same");
      addLegend(gPad);
   }

   // 2 — Track length
   drawPair(c->cd(2), geant.hTrackLength, simple.hTrackLength);
   addLegend(c->cd(2));

   // 3 — Total energy loss
   drawPair(c->cd(3), geant.hTotalELoss, simple.hTotalELoss);
   addLegend(c->cd(3));

   // 4 — XY (Geant4 or SimpleSim if no Geant4)
   c->cd(4); gPad->SetLeftMargin(0.15);
   (geant.nHits > 0 ? geant.hXY : simple.hXY)->Draw("colz");
   if (geant.nHits > 0) geant.hXY->SetTitle("Geant4 XY hits");
   else                  simple.hXY->SetTitle("SimpleSim XY hits");

   // 5 — XY (SimpleSim)
   c->cd(5); gPad->SetLeftMargin(0.15);
   simple.hXY->SetTitle("SimpleSim XY hits");
   simple.hXY->Draw("colz");

   // 6 — Z hit distribution
   drawPair(c->cd(6), geant.hHitZ, simple.hHitZ);
   addLegend(c->cd(6));

   c->SaveAs("./data/compareSimVsGeant.pdf");
   std::cout << "Saved ./data/compareSimVsGeant.pdf\n";
}
