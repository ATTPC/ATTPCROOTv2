/**
 * ELossComparison.C
 *
 * Comparative plots of the three energy loss models in ATTPCROOTv2:
 *   - AtELossBetheBloch  (analytic PDG Bethe-Bloch formula)
 *   - AtELossCATIMA      (CATIMA library wrapper)
 *   - AtELossTable       (SRIM lookup table with spline interpolation)
 *
 * Four scenarios are plotted:
 *   1. Proton      in H2 gas @ 600 Torr  (all three models)
 *   2. Deuteron    in D2 gas @ 600 Torr  (BB + CATIMA + Table)
 *   3. Alpha (4He) in D2 gas @ 600 Torr  (BB + CATIMA + Table)
 *   4. Proton      in He gas @ 700 Torr  (BB + CATIMA + Table)
 *
 * Each scenario produces a canvas with four pads:
 *   Top-left     dE/dx vs kinetic energy  (log-log)
 *   Top-right    Range  vs kinetic energy  (log-log)
 *   Bottom-left  Bragg curve              (dE/dx vs distance from entry)
 *   Bottom-right Range straggling sigma vs kinetic energy (log-log)
 *                  Note: Table straggling requires a full SRIM file with the
 *                  "Multiply" conversion section.  Simple SRIM tables (no
 *                  conversion header) are loaded without straggling data and
 *                  are omitted from the straggling pad.
 *
 * Usage (after sourcing build/config.sh):
 *   root -l -b -q ELossComparison.C
 *
 * Output: ELossComparison.pdf (four pages, one per scenario)
 *
 * Model notes
 * -----------
 * "Bethe-Bloch (Bohr)" — AtELossBetheBloch: pure PDG 2022 Eq. 34.1, bare projectile charge,
 *   Bohr range-straggling integral (dω²/dE = K·z²·(Z/A)·ρ·mₑ / |dEdx|³), √E extrapolation
 *   below the BB validity threshold.
 *
 * "CATIMA (default)" — AtELossCATIMA with catima::default_config:
 *   - Effective charge: Pierce-Blann (z_eff → 0 for protons at low β — suppresses dEdx and
 *     straggling proportionally to z_eff²).
 *   - dEdx: Bethe-Bloch + shell corrections + Barkas term + density effect + Lindhard correction;
 *     SRIM-85 tables at low energies.
 *   - Straggling: Bohr × Lindhard-X factor (quantum correction, always active); Firsov formula
 *     used as upper cap below 30 MeV/u — this is LOWER than pure Bohr and causes the straggling
 *     to drop steeply toward zero at low energy.
 *
 * "CATIMA (bare, no corr)" — AtELossCATIMA with custom config:
 *   - Effective charge: none (bare Z, matching BB).
 *   - dEdx corrections disabled: no Barkas, no Lindhard, no shell corrections.
 *   - Straggling: still Bohr × Lindhard-X + Firsov cap (not removable via config in the bundled
 *     CATIMA version).  Low-energy dEdx still from SRIM-85 tables, not √E.
 *   This is the closest CATIMA can get to a pure BB+Bohr calculation; any remaining gap versus the
 *   red BB curve is due to the Firsov straggling correction and SRIM-85 vs √E extrapolation.
 */

// ---------------------------------------------------------------------------
// Unit conversion helpers (energy -> MeV, length -> mm)
// ---------------------------------------------------------------------------
double UnitToMeV(const std::string &u)
{
   if (u == "eV")
      return 1e-6;
   if (u == "keV")
      return 1e-3;
   if (u == "MeV")
      return 1.0;
   if (u == "GeV")
      return 1e3;
   return 0.0;
}

double UnitToMM(const std::string &u)
{
   if (u == "um")
      return 1e-3;
   if (u == "mm")
      return 1.0;
   if (u == "cm")
      return 10.0;
   if (u == "m")
      return 1000.0;
   return 0.0;
}

// ---------------------------------------------------------------------------
// LoadSimpleSrimTable
//
// Load a SRIM-format stopping table that has the standard column layout but
// lacks the "Target Density" header and "Multiply" conversion section.
//
// SRIM stores dE/dx in MeV/(mg/cm2).  With known density rho (g/cm3):
//   dEdx [MeV/mm] = dEdx [MeV/(mg/cm2)] x rho [g/cm3] x 100
//
// Range straggling cannot be loaded because AtELossTable::LoadRangeVariance
// is private; the returned model has no straggling data.
// ---------------------------------------------------------------------------
AtTools::AtELossTable *LoadSimpleSrimTable(const std::string &fileName, double density)
{
   std::ifstream file(fileName);
   if (!file.is_open()) {
      ::Error("LoadSimpleSrimTable", "Cannot open file: %s", fileName.c_str());
      return nullptr;
   }

   const double conversion = density * 100.0; // MeV/(mg/cm2) -> MeV/mm

   std::vector<double> energy;
   std::vector<double> dEdX;

   std::string line;
   while (std::getline(file, line)) {
      std::istringstream iss(line);
      std::vector<std::string> tok;
      std::string t;
      while (iss >> t)
         tok.push_back(t);

      // Need: energy Eunit dEdx_e dEdx_n range Runit strag Sunit  (>=8 tokens)
      if (tok.size() < 8)
         continue;

      double en, dedxE, dedxN;
      try {
         en = std::stod(tok[0]);
         dedxE = std::stod(tok[2]);
         dedxN = std::stod(tok[3]);
      } catch (...) {
         continue;
      }

      double enMeV = en * UnitToMeV(tok[1]);
      double dedxMeVmm = (dedxE + dedxN) * conversion;

      if (enMeV <= 0.0 || dedxMeVmm <= 0.0)
         continue;

      energy.push_back(enMeV);
      dEdX.push_back(dedxMeVmm);
   }

   if (energy.empty()) {
      ::Error("LoadSimpleSrimTable", "No valid rows parsed from %s", fileName.c_str());
      return nullptr;
   }

   return new AtTools::AtELossTable(energy, dEdX, density);
}

// ---------------------------------------------------------------------------
// Graph-filling helpers: sample a model over a log-spaced energy grid
// ---------------------------------------------------------------------------
TGraph *MakeDedxGraph(AtTools::AtELossModel *m, double eMin, double eMax, int nPts = 200)
{
   auto *g = new TGraph(nPts);
   for (int i = 0; i < nPts; ++i) {
      double e = eMin * TMath::Power(eMax / eMin, double(i) / (nPts - 1));
      g->SetPoint(i, e, m->GetdEdx(e));
   }
   return g;
}

TGraph *MakeRangeGraph(AtTools::AtELossModel *m, double eMin, double eMax, int nPts = 200)
{
   auto *g = new TGraph(nPts);
   for (int i = 0; i < nPts; ++i) {
      double e = eMin * TMath::Power(eMax / eMin, double(i) / (nPts - 1));
      g->SetPoint(i, e, m->GetRange(e));
   }
   return g;
}

// hasStrag: pass false for table models loaded without straggling data
TGraph *MakeStragGraph(AtTools::AtELossModel *m, double eMin, double eMax, bool hasStrag, int nPts = 200)
{
   auto *g = new TGraph();
   if (!hasStrag)
      return g;
   for (int i = 0; i < nPts; ++i) {
      double e = eMin * TMath::Power(eMax / eMin, double(i) / (nPts - 1));
      double s = m->GetRangeStraggling(e);
      if (s > 0.0)
         g->AddPoint(e, s);
   }
   return g;
}

TGraph *MakeBraggGraph(AtTools::AtELossModel *m, double energy, double stepMM = 0.5)
{
   auto pts = m->GetBraggCurve(energy, stepMM);
   auto *g = new TGraph(int(pts.size()));
   for (int i = 0; i < int(pts.size()); ++i)
      g->SetPoint(i, pts[i].second, pts[i].first);
   return g;
}

// ---------------------------------------------------------------------------
// ModelEntry: bundles a model pointer with display properties
// ---------------------------------------------------------------------------
struct ModelEntry {
   AtTools::AtELossModel *model;
   std::string name;
   int color;
   int lineStyle;
   bool hasStraggling;
};

// ---------------------------------------------------------------------------
// DrawScenario: fills a pre-divided 4-pad canvas for one particle/material
// ---------------------------------------------------------------------------
void DrawScenario(const std::string &title, const std::vector<ModelEntry> &entries, double eMin, double eMax,
                  double braggEnergy, TCanvas *c)
{
   c->SetTitle(title.c_str());
   c->Divide(2, 2, 0.005, 0.005);

   auto StyleGraph = [](TGraph *g, int col, int sty) {
      g->SetLineColor(col);
      g->SetLineStyle(sty);
      g->SetLineWidth(2);
   };

   // --- Pad 1: dE/dx vs E ---
   {
      c->cd(1)->SetLogx();
      c->cd(1)->SetLogy();
      c->cd(1)->SetGrid();
      auto *mg = new TMultiGraph();
      auto *leg = new TLegend(0.55, 0.65, 0.88, 0.88);
      leg->SetBorderSize(0);
      for (auto &e : entries) {
         auto *g = MakeDedxGraph(e.model, eMin, eMax);
         StyleGraph(g, e.color, e.lineStyle);
         mg->Add(g, "L");
         leg->AddEntry(g, e.name.c_str(), "L");
      }
      mg->SetTitle(Form("%s;Kinetic energy (MeV);dE/dx (MeV/mm)", title.c_str()));
      mg->Draw("A");
      leg->Draw();
   }

   // --- Pad 2: Range vs E ---
   {
      c->cd(2)->SetLogx();
      c->cd(2)->SetLogy();
      c->cd(2)->SetGrid();
      auto *mg = new TMultiGraph();
      auto *leg = new TLegend(0.15, 0.65, 0.48, 0.88);
      leg->SetBorderSize(0);
      for (auto &e : entries) {
         auto *g = MakeRangeGraph(e.model, eMin, eMax);
         StyleGraph(g, e.color, e.lineStyle);
         mg->Add(g, "L");
         leg->AddEntry(g, e.name.c_str(), "L");
      }
      mg->SetTitle(";Kinetic energy (MeV);Range (mm)");
      mg->Draw("A");
      leg->Draw();
   }

   // --- Pad 3: Bragg curve ---
   {
      c->cd(3)->SetGrid();
      auto *mg = new TMultiGraph();
      auto *leg = new TLegend(0.15, 0.65, 0.48, 0.88);
      leg->SetBorderSize(0);
      for (auto &e : entries) {
         auto *g = MakeBraggGraph(e.model, braggEnergy);
         StyleGraph(g, e.color, e.lineStyle);
         mg->Add(g, "L");
         leg->AddEntry(g, e.name.c_str(), "L");
      }
      mg->SetTitle(Form("Bragg curve (E_{0} = %.1f MeV);Distance from entry (mm);dE/dx (MeV/mm)", braggEnergy));
      mg->Draw("A");
      leg->Draw();
   }

   // --- Pad 4: Range straggling vs E ---
   {
      c->cd(4)->SetLogx();
      c->cd(4)->SetLogy();
      c->cd(4)->SetGrid();
      auto *mg = new TMultiGraph();
      auto *leg = new TLegend(0.15, 0.65, 0.48, 0.88);
      leg->SetBorderSize(0);
      bool anyDrawn = false;
      for (auto &e : entries) {
         auto *g = MakeStragGraph(e.model, eMin, eMax, e.hasStraggling);
         if (g->GetN() == 0) {
            delete g;
            continue;
         }
         StyleGraph(g, e.color, e.lineStyle);
         mg->Add(g, "L");
         leg->AddEntry(g, e.name.c_str(), "L");
         anyDrawn = true;
      }
      if (anyDrawn) {
         mg->SetTitle(";Kinetic energy (MeV);Range straggling #sigma (mm)");
         mg->Draw("A");
      }
      leg->Draw();
   }
}

// ===========================================================================
// Main macro
// ===========================================================================
void ELossComparison()
{
   TString vmcDir = getenv("VMCWORKDIR") ? getenv("VMCWORKDIR") : "../../..";
   TString resDir = vmcDir + "/resources/energy_loss";

   // -------------------------------------------------------------------------
   // Physical parameters
   // -------------------------------------------------------------------------

   // Particle rest masses (MeV/c2) for Bethe-Bloch
   const double kMproton = 938.272;
   const double kMdeuteron = 1875.61;
   const double kMalpha = 3727.38;

   // Particle masses in amu for CATIMA
   const double kMproton_amu = 1.00728;
   const double kMdeuteron_amu = 2.01355;
   const double kMalpha_amu = 4.00151;

   // Gas densities (g/cm3) at stated pressures, room temperature (~20 C)
   const double kRho_H2_600 = 6.5643e-5; // H2  @ 600 Torr (CATIMA/LISE reference tests)
   const double kRho_D2_600 = 1.313e-4;  // D2  @ 600 Torr (scaled by molar-mass ratio from H2)
   const double kRho_He_700 = 1.53e-4;   // He  @ 700 Torr (He STP x 700/760 x 273/293)

   // Mean excitation energies (eV) -- PDG 2022 Table 34.1
   const double kI_H = 19.2;
   const double kI_He = 41.8;

   // =========================================================================
   // Scenario 1: Proton in H2 @ 600 Torr
   //   All three models.  HinH.txt has the full SRIM format (density + Multiply)
   //   so the table model includes straggling data.
   // =========================================================================
   auto bb_p_H2 = new AtTools::AtELossBetheBloch(1.0, kMproton, 1, 1, kRho_H2_600, kI_H);

   auto catima_p_H2 =
      new AtTools::AtELossCATIMA(kRho_H2_600, std::vector<std::tuple<int, int, int>>{{1, 1, 1}});
   catima_p_H2->SetProjectile(1, 1, kMproton_amu);

   auto table_p_H2 = new AtTools::AtELossTable(0);
   table_p_H2->LoadSrimTable(Form("%s/HinH.txt", resDir.Data()));
   table_p_H2->SetDensity(kRho_H2_600);

   // =========================================================================
   // Scenario 2: Deuteron in D2 @ 600 Torr
   // =========================================================================
   auto bb_d_D2 = new AtTools::AtELossBetheBloch(1.0, kMdeuteron, 1, 2, kRho_D2_600, kI_H);

   auto catima_d_D2 =
      new AtTools::AtELossCATIMA(kRho_D2_600, std::vector<std::tuple<int, int, int>>{{2, 1, 1}});
   catima_d_D2->SetProjectile(2, 1, kMdeuteron_amu);

   auto table_d_D2 = LoadSimpleSrimTable(Form("%s/deuteron_D2_600torr.txt", resDir.Data()), kRho_D2_600);

   // =========================================================================
   // Scenario 3: Alpha (4He) in D2 @ 600 Torr
   // =========================================================================
   auto bb_a_D2 = new AtTools::AtELossBetheBloch(2.0, kMalpha, 1, 2, kRho_D2_600, kI_H);

   auto catima_a_D2 =
      new AtTools::AtELossCATIMA(kRho_D2_600, std::vector<std::tuple<int, int, int>>{{2, 1, 1}});
   catima_a_D2->SetProjectile(4, 2, kMalpha_amu);

   auto table_a_D2 = LoadSimpleSrimTable(Form("%s/alpha_D2_600torr.txt", resDir.Data()), kRho_D2_600);

   // =========================================================================
   // Scenario 4: Proton in He @ 700 Torr
   // =========================================================================
   auto bb_p_He = new AtTools::AtELossBetheBloch(1.0, kMproton, 2, 4, kRho_He_700, kI_He);

   auto catima_p_He =
      new AtTools::AtELossCATIMA(kRho_He_700, std::vector<std::tuple<int, int, int>>{{4, 2, 1}});
   catima_p_He->SetProjectile(1, 1, kMproton_amu);

   auto table_p_He = LoadSimpleSrimTable(Form("%s/proton_He_700torr.txt", resDir.Data()), kRho_He_700);

   // =========================================================================
   // "CATIMA bare" config: bare charge + no dEdx corrections — closest analog
   // to the pure Bethe-Bloch model available within the CATIMA library.
   // Note: the Lindhard-X / Firsov straggling correction is always active in
   // the bundled CATIMA source and cannot be disabled via Config.
   // =========================================================================
   catima::Config pureCfg;
   pureCfg.z_effective = catima::z_eff_type::none; // bare Z, not Pierce-Blann
   pureCfg.corrections =
      catima::no_barkas | catima::no_lindhard | catima::no_shell_correction;

   auto catimaPure_p_H2 =
      new AtTools::AtELossCATIMA(kRho_H2_600, std::vector<std::tuple<int, int, int>>{{1, 1, 1}});
   catimaPure_p_H2->SetProjectile(1, 1, kMproton_amu);
   catimaPure_p_H2->SetConfig(pureCfg);

   auto catimaPure_d_D2 =
      new AtTools::AtELossCATIMA(kRho_D2_600, std::vector<std::tuple<int, int, int>>{{2, 1, 1}});
   catimaPure_d_D2->SetProjectile(2, 1, kMdeuteron_amu);
   catimaPure_d_D2->SetConfig(pureCfg);

   auto catimaPure_a_D2 =
      new AtTools::AtELossCATIMA(kRho_D2_600, std::vector<std::tuple<int, int, int>>{{2, 1, 1}});
   catimaPure_a_D2->SetProjectile(4, 2, kMalpha_amu);
   catimaPure_a_D2->SetConfig(pureCfg);

   auto catimaPure_p_He =
      new AtTools::AtELossCATIMA(kRho_He_700, std::vector<std::tuple<int, int, int>>{{4, 2, 1}});
   catimaPure_p_He->SetProjectile(1, 1, kMproton_amu);
   catimaPure_p_He->SetConfig(pureCfg);

   // =========================================================================
   // Color / line-style scheme
   // =========================================================================
   const int kColBB = kRed + 1;
   const int kColCATIMA = kBlue + 1;
   const int kColTable = kGreen + 2;
   const int kColCATIMApure = kMagenta + 1;

   // =========================================================================
   // Draw and save
   // =========================================================================
   auto *c1 = new TCanvas("c1", "Proton in H2", 1200, 900);
   {
      std::vector<ModelEntry> ents = {
         {bb_p_H2, "Bethe-Bloch (Bohr)", kColBB, 1, true},
         {catima_p_H2, "CATIMA (default)", kColCATIMA, 2, true},
         {table_p_H2, "SRIM Table", kColTable, 3, true},
         {catimaPure_p_H2, "CATIMA (bare, no corr)", kColCATIMApure, 7, true},
      };
      DrawScenario("Proton in H_{2} (600 Torr)", ents, 0.1, 10.0, 5.0, c1);
   }

   auto *c2 = new TCanvas("c2", "Deuteron in D2", 1200, 900);
   {
      std::vector<ModelEntry> ents = {
         {bb_d_D2, "Bethe-Bloch (Bohr)", kColBB, 1, true},
         {catima_d_D2, "CATIMA (default)", kColCATIMA, 2, true},
         {catimaPure_d_D2, "CATIMA (bare, no corr)", kColCATIMApure, 7, true},
      };
      if (table_d_D2)
         ents.push_back({table_d_D2, "SRIM Table", kColTable, 3, false});
      DrawScenario("Deuteron in D_{2} (600 Torr)", ents, 0.2, 20.0, 10.0, c2);
   }

   auto *c3 = new TCanvas("c3", "Alpha in D2", 1200, 900);
   {
      std::vector<ModelEntry> ents = {
         {bb_a_D2, "Bethe-Bloch (Bohr)", kColBB, 1, true},
         {catima_a_D2, "CATIMA (default)", kColCATIMA, 2, true},
         {catimaPure_a_D2, "CATIMA (bare, no corr)", kColCATIMApure, 7, true},
      };
      if (table_a_D2)
         ents.push_back({table_a_D2, "SRIM Table", kColTable, 3, false});
      DrawScenario("Alpha (^{4}He) in D_{2} (600 Torr)", ents, 0.5, 30.0, 10.0, c3);
   }

   auto *c4 = new TCanvas("c4", "Proton in He", 1200, 900);
   {
      std::vector<ModelEntry> ents = {
         {bb_p_He, "Bethe-Bloch (Bohr)", kColBB, 1, true},
         {catima_p_He, "CATIMA (default)", kColCATIMA, 2, true},
         {catimaPure_p_He, "CATIMA (bare, no corr)", kColCATIMApure, 7, true},
      };
      if (table_p_He)
         ents.push_back({table_p_He, "SRIM Table", kColTable, 3, false});
      DrawScenario("Proton in ^{4}He (700 Torr)", ents, 0.1, 10.0, 5.0, c4);
   }

   c1->Print("ELossComparison.pdf(");
   c2->Print("ELossComparison.pdf");
   c3->Print("ELossComparison.pdf");
   c4->Print("ELossComparison.pdf)");

   ::Info("ELossComparison", "Saved to ELossComparison.pdf");
}
