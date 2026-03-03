/**
 * Plot the Bethe-Bloch electronic stopping power of protons in aluminum using
 * AtELossBetheBloch, reproducing the "pure Bethe" curve from the PDG reference plot.
 *
 * Aluminum parameters (PDG 2022):
 *   Z = 13, A = 27, density = 2.700 g/cm³, I = 166 eV
 *
 * Run with:
 *   root -l plot_bethe_bloch_Al.C
 */

void plot_bethe_bloch_Al()
{
   gSystem->Load("libAtTools.so");

   // ── Aluminum material parameters (PDG 2022) ─────────────────────────────
   const int mat_Z = 13;
   const int mat_A = 27;
   const double density = 2.700; // g/cm³
   const double I_eV = 166.0;   // mean excitation energy, eV

   // ── Proton projectile ───────────────────────────────────────────────────
   const double part_q = 1.0;
   const double part_mass = 938.272; // MeV/c²

   // Build model with a dense grid tuned to 0.1 – 100 MeV
   AtTools::AtELossBetheBloch model(part_q, part_mass, mat_Z, mat_A, density, I_eV);
   model.BuildSpline(0.05, 200.0, 500);

   // ── Sample dEdx on a log-spaced energy grid ──────────────────────────────
   const int nPoints = 300;
   const double eMin = 0.1;  // MeV
   const double eMax = 100.0; // MeV

   TGraph *gBB = new TGraph(nPoints);
   gBB->SetName("gBB");
   gBB->SetTitle("Proton stopping power in Al (pure Bethe-Bloch)");

   for (int i = 0; i < nPoints; ++i) {
      double t = static_cast<double>(i) / (nPoints - 1);
      double energy = eMin * TMath::Power(eMax / eMin, t); // log-spaced
      double dedx = model.GetdEdx(energy);                 // MeV/mm
      gBB->SetPoint(i, energy, dedx);
   }

   // ── Canvas and style ─────────────────────────────────────────────────────
   gStyle->SetOptStat(0);
   gStyle->SetPadGridX(0);
   gStyle->SetPadGridY(0);

   TCanvas *c = new TCanvas("c_BB", "Bethe-Bloch: H in Al", 900, 700);
   c->SetLogx();
   c->SetLogy();
   c->SetLeftMargin(0.13);
   c->SetBottomMargin(0.12);

   // Draw a blank frame first to set axis ranges
   TH1F *frame = c->DrawFrame(eMin, 1.0, eMax, 200.0);
   frame->GetXaxis()->SetTitle("Energy [MeV]");
   frame->GetYaxis()->SetTitle("Electronic Stopping Power [MeV/mm]");
   frame->GetXaxis()->SetTitleSize(0.05);
   frame->GetYaxis()->SetTitleSize(0.05);
   frame->GetXaxis()->SetLabelSize(0.045);
   frame->GetYaxis()->SetLabelSize(0.045);
   frame->GetYaxis()->SetTitleOffset(1.2);

   gBB->SetLineColor(kRed);
   gBB->SetLineWidth(2);
   gBB->Draw("L same");

   TLegend *leg = new TLegend(0.55, 0.65, 0.88, 0.80);
   leg->SetBorderSize(1);
   leg->SetFillStyle(1001);
   leg->AddEntry(gBB, "Pure Bethe-Bloch (AtELossBetheBloch)", "l");
   leg->Draw();

   TLatex *info = new TLatex();
   info->SetNDC();
   info->SetTextSize(0.035);
   info->DrawLatex(0.55, 0.58, "Proton in Al  (Z=13, A=27)");
   info->DrawLatex(0.55, 0.53, "#rho = 2.70 g/cm^{3},  I = 166 eV");

   c->Update();
   c->SaveAs("proton_stopping_Al_BetheBloch.pdf");
}
