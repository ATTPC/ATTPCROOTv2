#include <TString.h>

#include "../unpack/TxtEvent.h"
/**
 * Tools for filling just a bunch of different plots from the MCResult of the fit.
 *
 */
TH1F *zHist = nullptr;
TH1F *aHist = nullptr;
TH1F *hAmp = nullptr;
TH1F *hObj = nullptr;
TH1F *hObjPos = nullptr;
TH1F *hObjQ = nullptr;
TH1F *hMR = nullptr;

TH2F *hBeam = nullptr;

TH2F *hZvsObj = nullptr;
TH2F *hZvsAmp = nullptr;
TH2F *hAmpvsPosObj = nullptr;
TH2F *hAmpvsObj = nullptr;
TH2F *hAmpvsLoc = nullptr;
TChain *tree;
MCFitter::AtMCResult *result = nullptr;

int maxObjQ = 100;

double ex(double E)
{
   return E;
   int ABeam = 200;
   // Ecm
   double Ecm = E / ABeam * 4;

   // Q Value (200Bi + 4He -> 204At)
   double Q = -6.07;
   return Ecm + Q;
}

double GetAvg(string name, TClonesArray &result, int toAvg = 1)
{
   double num = 0;
   double denom = 0;
   for (int i = 0; i < toAvg; ++i) {
      auto res = dynamic_cast<MCFitter::AtMCResult *>(result.At(i));
      auto w = 1 / res->fObjective;
      num += res->fParameters[name] * w;
      denom += w;
   }
   return num / denom;
}

void FillPlots(float ampMin = 0, float ampCut = 1, float qMin = 0, float qMax = 1000, float EMin = 0, float EMax = 5000,
               float pos = 100)
{

   TTreeReader reader(tree);
   TTreeReaderValue<TClonesArray> resultArray(reader, "AtMCResult");
   zHist->Reset();
   aHist->Reset();
   hAmp->Reset();
   hObj->Reset();
   hBeam->Reset();
   hObjPos->Reset();
   hObjQ->Reset();
   hZvsObj->Reset();
   hZvsAmp->Reset();
   hAmpvsPosObj->Reset();
   hAmpvsObj->Reset();
   hAmpvsLoc->Reset();
   hMR->Reset();

   TxtEvents check;
   check.AddTxtFile("./data/Chi2QCut13.txt");
   ofstream file("goodEvent.txt");

   while (reader.Next() && reader.GetCurrentEntry() < 10000) {

      if (true || reader.GetCurrentEntry() == 17) {
         if (false && !check(reader.GetCurrentEntry()))
            continue;

         result = dynamic_cast<MCFitter::AtMCResult *>(resultArray->At(0));

         int z = result->fParameters["Z1"];
         int z2 = result->fParameters["Z2"];
         float amp = result->fParameters["Amp"];
         float objPos = result->fParameters["ObjPos"];
         float objQ = result->fParameters["ObjQ"];

         for (int i = 0; i < resultArray->GetEntries(); ++i) {

            result = dynamic_cast<MCFitter::AtMCResult *>(resultArray->At(i));

            if (objPos < pos && amp < ampCut && amp > ampMin && objQ < qMax && objQ > qMin) {

               hZvsObj->Fill(z - result->fParameters["Z1"], result->fParameters["ObjQ"]);
               hZvsObj->Fill(-(z - result->fParameters["Z1"]), result->fParameters["ObjQ"]);
               // hZvsObj->Fill(z2 - result->fParameters["Z2"], result->fParameters["ObjQ"]);
               hZvsAmp->Fill(z - result->fParameters["Z1"], result->fParameters["Amp"]);
               hAmpvsPosObj->Fill(result->fParameters["Amp"], result->fParameters["ObjPos"]);
            }
         }
      }

      result = dynamic_cast<MCFitter::AtMCResult *>(resultArray->At(0));
      if (result) {
         if (ex(result->fParameters["EBeam"]) > EMin && ex(result->fParameters["EBeam"]) < EMax &&
             result->fParameters["Amp"] < ampCut && result->fParameters["Amp"] > ampMin &&
             result->fParameters["ObjQ"] < qMax && result->fParameters["ObjQ"] > qMin &&
             result->fParameters["ObjPos"] < pos) {

            // Save event number to file
            file << reader.GetCurrentEntry() << std::endl;

            // zHist->Fill(result->fParameters["Z0"]);
            // zHist->Fill(result->fParameters["Z1"]);
            zHist->Fill(GetAvg("Z0", *resultArray, 10));
            zHist->Fill(GetAvg("Z1", *resultArray, 10));

            hMR->Fill(result->fParameters["Z0"] / (result->fParameters["Z0"] + result->fParameters["Z1"]));
            hMR->Fill(result->fParameters["Z1"] / (result->fParameters["Z0"] + result->fParameters["Z1"]));
            hAmpvsObj->Fill(result->fParameters["Amp"], result->fParameters["ObjQ"]);

            aHist->Fill(result->fParameters["A0"]);
            aHist->Fill(result->fParameters["A1"]);
            hAmp->Fill(result->fParameters["Amp"]);
            hObj->Fill(result->fObjective);
            hObjPos->Fill(result->fParameters["ObjPos"]);
            hObjQ->Fill(result->fParameters["ObjQ"]);
            // Plot angle1 vs angle2
            hBeam->Fill(result->fParameters["vZ"], ex(result->fParameters["EBeam"]));
            hAmpvsLoc->Fill(result->fParameters["vZ"], result->fParameters["Amp"]);
         }
      }
   }
}

void PlotRange(double min, double max, double step)
{
   // Pictures will be saved to ./figures/out
   TCanvas *c1 = new TCanvas();

   int numIter = (max - min) / step;
   for (int i = 0; i < numIter; ++i) {

      double lMin = min + i * step;
      double lMax = lMin + step;
      cout << i << " " << lMin << " to " << lMax << endl;

      FillPlots(0, 1, lMin, lMax);
      c1->cd();
      hZvsAmp->Draw("colz");
      c1->SaveAs(TString::Format("./figures/out/ZvsAmp%f_%f.png", lMin, lMax));
   }
}

void PlotZ(bool draw = true)
{
   TH1::SetDefaultSumw2();
   // TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/cut1/SRIM/Bi200Diff2.root";
   TString fileName = "/mnt/analysis/e12014/TPC/150Torr_yFit/cut1/SRIM/Bi200Chi2.root";
   //  TString fileName = "/mnt/analysis/e12014/TPC/150Torr_yFit/cut1/LISE/Bi200Chi2.root";
   //  TString fileName = "/mnt/analysis/e12014/TPC/150Torr_yFit2/cut1/SRIM/Bi200Chi2.root";
   //    TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/cut1/SRIM/Bi200Chi2.root";
   //       TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/cut2/SRIM/Bi200Chi2.root";
   //         TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/cut1/SRIM/Pb198Chi2.root";
   //                TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/sameV/cut1/SRIM/Bi200Diff2.root";
   //                TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/pConserve/SRIM/Bi200Chi2.root";
   //            TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/sameVSmall/cut1/SRIM/Bi200Chi2.root";
   //            TString fileName = "./Chi2FixAmp.root";
   //          TString fileName = "./Chi2NormalPosition.root";
   //        TString fileName = "./Bi200NewObj.root";
   //        TString fileName = "./Chi2Norm.root";
   //    TString fileName = "./Bi200NewFit.root";

   if (!tree) {
      tree = new TChain("cbmsim");
      tree->Add(fileName);
   }

   int zMin = 26;
   int zMax = 59;
   int aMin = zMin * (float)204 / 85;
   int aMax = zMax * (float)204 / 85;
   zHist = new TH1F("hZ", "Z", zMax - zMin + 1, zMin - 0.5, zMax + 0.5);
   hMR = new TH1F("hMR", "M_R", zMax - zMin + 1, (zMin - 0.5) / 85, (zMax + 0.5) / 85);
   aHist = new TH1F("hA", "A", zMax - zMin + 1, aMin, aMax);
   hBeam = new TH2F("hBeam", "Beam energy", 100, 0, 1000, 100, 0, ex(4500));
   hAmp = new TH1F("hAmp", "Charge Scaling Factor", 100, 0, 1);
   hObj = new TH1F("hObj", "Objective Function", 100, 0, maxObjQ);
   hObjPos = new TH1F("hObjPos", "Objective Function Position", 50, 0, 50);
   hObjQ = new TH1F("hObjQ", "Objective Function Charge", 100, 0, maxObjQ);

   hZvsObj = new TH2F("hZvsObj", "dZ vs Chi2", 21, -10 - .5, 10.5, 100, 0, maxObjQ);
   hZvsAmp = new TH2F("hZvsAmp", "dZ vs Amp", 20 + 1, -10, 10, 50, 0, 1);
   hAmpvsPosObj = new TH2F("hAmpvsPosObj", "Amp vs Pos Objective", 50, 0, 1, 50, 0, 10);
   hAmpvsObj = new TH2F("hAmpvsObj", "Amp vs Objective", 25, 0.3, .8, 25, 0, maxObjQ / 2.);
   hAmpvsLoc = new TH2F("hAmpvsLoc", "Amp vs Location", 50, 0, 1000, 50, 0, 1);
   FillPlots();
   if (draw)
      zHist->Draw();
}

void FitZ()
{
   // Fit two functions, gaus and gaus2.
   zHist->SetStats(0);
   zHist->SetMarkerStyle(22);
   zHist->SetMarkerSize(2);
   gStyle->SetErrorX(0);
   zHist->Draw("ep");

   TF1 *gaus1 = new TF1("gaus1", "gaus(0)", 30, 55);
   // TF1 *gaus1 = new TF1("gaus1", "gaus(0)", 43, 55);
   std::cout << "Fitting single gaussian" << endl;
   gaus1->SetParameters(80, 42.5, 5);
   gaus1->FixParameter(1, 42.5);

   zHist->Fit(gaus1, "SNR");
   gaus1->SetLineColor(kBlue);
   gaus1->SetLineStyle(2);
   gaus1->DrawCopy("same");

   /*
   TF1 *gaus2 =
      new TF1("gaus2", " [0]*( exp( -(x - [1] - [3])^2/(2*[2]^2)) + exp( -(x - [1]+[3])^2/(2*[2]^2)))", 26, 59);
   gaus2->SetParameters(gaus1->GetParameter(0) / 2, 42.5, 5, 2);
   gaus2->FixParameter(1, 42.5);
   // gaus2->SetParLimits(3, 2, 10);
   std::cout << "Fitting double gaussian" << endl;
   auto res = zHist->Fit(gaus2, "SN");

   gaus1->SetParameters(res->Parameter(0), res->Parameter(1) - res->Parameter(3), res->Parameter(2));
   gaus1->SetLineColor(kGreen);
   gaus1->SetLineStyle(7);
   gaus1->DrawCopy("same");
   gaus1->SetParameters(res->Parameter(0), res->Parameter(1) + res->Parameter(3), res->Parameter(2));
   gaus1->DrawCopy("same");
   gaus2->Draw("same");
   */
}

void FitMR()
{
   // Fit two functions, gaus and gaus2.
   hMR->SetStats(0);
   hMR->SetMarkerStyle(22);
   hMR->SetMarkerSize(2);
   gStyle->SetErrorX(0);
   hMR->Draw("ep");

   TF1 *gaus2 =
      new TF1("gaus2", " [0]*( exp( -(x - [1] - [3])^2/(2*[2]^2)) + exp( -(x - [1]+[3])^2/(2*[2]^2)))", 26, 59);
   gaus2->SetParameters(80, 0.5, 0.06, 2);
   gaus2->FixParameter(1, 0.5);
   std::cout << "Fitting double gaussian" << endl;
   auto res = hMR->Fit(gaus2, "SN");

   TF1 *gaus1 = new TF1("gaus1", "gaus(0)", 26, 59);
   gaus1->SetParameters(res->Parameter(0), res->Parameter(1) - res->Parameter(3), res->Parameter(2));
   gaus1->SetLineColor(kGreen);
   gaus1->SetLineStyle(7);
   gaus1->DrawCopy("same");
   gaus1->SetParameters(res->Parameter(0), res->Parameter(1) + res->Parameter(3), res->Parameter(2));
   gaus1->DrawCopy("same");
   gaus2->Draw("same");

   std::cout << "Fitting single gaussian" << endl;
   gaus1->SetParameters(80, 0.5, 0.06);
   gaus1->FixParameter(1, 0.5);

   hMR->Fit(gaus1, "SN");
   gaus1->SetLineColor(kBlue);
   gaus1->SetLineStyle(2);
   gaus1->DrawCopy("same");
}

void FitTheory(string fileName)
{
   std::ifstream file(fileName);
   std::vector<double> A;
   std::vector<double> Yield;
   for (auto &row : CSVRange<double>(file)) {
      A.push_back(row[0]);
      Yield.push_back(row[1]);
   }
   auto g = new TGraph(A.size(), A.data(), Yield.data());
   g->Fit("gaus");
   g->Draw("AC*");
}

void FitTheoryMumpower(string fileName)
{
   std::ifstream file(fileName);
   std::string header;
   std::getline(file, header);

   std::vector<double> Z;
   std::vector<double> A;
   std::vector<double> Yield;
   for (auto &row : CSVRange<double>(file)) {
      Z.push_back(row[0]);
      A.push_back(row[1]);
      Yield.push_back(row[2]);
   }
   std::vector<double> ZComp;
   std::vector<double> YieldComp;
   int lastZ = Z[0];
   ZComp.push_back(0);
   YieldComp.push_back(0);
   for (int i = 0; i < Z.size(); ++i) {
      if (lastZ == Z[i])
         YieldComp.back() += Yield[i];
      else {
         ZComp.push_back(Z[i]);
         YieldComp.push_back(Yield[i]);
         lastZ = Z[i];
      }
   }
   auto g = new TGraph(ZComp.size(), ZComp.data(), YieldComp.data());
   g->Fit("gaus");
   g->Draw("AC*");
}
