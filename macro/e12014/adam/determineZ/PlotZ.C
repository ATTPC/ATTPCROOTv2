#include <TString.h>

TH1F *zHist = nullptr;
TH1F *aHist = nullptr;
TH1F *hAmp = nullptr;
TH1F *hObj = nullptr;
TH1F *hObjPos = nullptr;
TH1F *hObjQ = nullptr;

TH2F *hBeam = nullptr;

TH2F *hZvsObj = nullptr;
TH2F *hZvsAmp = nullptr;
TH2F *hAmpvsPosObj = nullptr;
TH2F *hAmpvsObj = nullptr;
TH2F *hAmpvsLoc = nullptr;
TChain *tree;
MCFitter::AtMCResult *result = nullptr;

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

void FillPlots(float ampCut = 1, float EMin = 0, float EMax = 5000, float pos = 100)
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

   while (reader.Next() && reader.GetCurrentEntry() < 10000) {

      // if (reader.GetCurrentEntry() == 36) {
      int z = 0;
      float amp = 0;
      for (int i = 0; i < resultArray->GetEntries(); ++i) {

         result = dynamic_cast<MCFitter::AtMCResult *>(resultArray->At(i));

         if (i == 0) {
            z = result->fParameters["Z1"];
            amp = result->fParameters["Amp"];
         }

         if (amp > 0 && amp < ampCut) {
            hZvsObj->Fill(z - result->fParameters["Z1"], result->fParameters["ObjQ"]);
            hZvsAmp->Fill(z - result->fParameters["Z1"], result->fParameters["Amp"]);
            hAmpvsPosObj->Fill(result->fParameters["Amp"], result->fParameters["ObjPos"]);
            hAmpvsObj->Fill(result->fParameters["Amp"], result->fParameters["ObjQ"]);
         }
      }
      //}

      result = dynamic_cast<MCFitter::AtMCResult *>(resultArray->At(0));
      if (result) {
         if (result->fParameters["vZ"] > EMin && result->fParameters["vZ"] < EMax &&
             result->fParameters["ObjPos"] < pos && result->fParameters["Amp"] < ampCut) {
            zHist->Fill(result->fParameters["Z0"]);
            zHist->Fill(result->fParameters["Z1"]);
            // zHist->Fill(GetAvg("Z0", *resultArray));
            // zHist->Fill(GetAvg("Z1", *resultArray));

            aHist->Fill(result->fParameters["A0"]);
            aHist->Fill(result->fParameters["A1"]);
            hAmp->Fill(result->fParameters["Amp"]);
            hObj->Fill(result->fObjective);
            hObjPos->Fill(result->fParameters["ObjPos"]);
            hObjQ->Fill(result->fParameters["ObjQ"]);
            // Plot angle1 vs angle2
            hBeam->Fill(result->fParameters["vZ"], result->fParameters["EBeam"]);
            hAmpvsLoc->Fill(result->fParameters["vZ"], result->fParameters["Amp"]);
         }
      }
   }
}

void PlotZ(bool draw = true)
{
   // TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/cut1/SRIM/Bi200Diff2.root";
   // TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/cut1/SRIM/Bi200Chi2.root";
   TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/cut2/SRIM/Bi200Chi2.root";
   // TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/cut1/SRIM/Pb198Chi2.root";
   //        TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/sameV/cut1/SRIM/Bi200Diff2.root";
   //        TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/pConserve/SRIM/Bi200Chi2.root";
   //    TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/sameVSmall/cut1/SRIM/Bi200Chi2.root";
   //    TString fileName = "./Chi2FixAmp.root";
   //  TString fileName = "./Chi2NormalPosition.root";

   if (!tree) {
      tree = new TChain("cbmsim");
      tree->Add(fileName);
   }

   int zMin = 31;
   int zMax = 55;
   int aMin = zMin * (float)204 / 85;
   int aMax = zMax * (float)204 / 85;
   zHist = new TH1F("hZ", "Z", zMax - zMin, zMin, zMax);
   aHist = new TH1F("hA", "A", zMax - zMin + 1, aMin, aMax);
   hBeam = new TH2F("hBeam", "Beam energy", 100, 0, 1000, 100, 0, 4500);
   hAmp = new TH1F("hAmp", "Charge Scaling Factor", 100, 0, 1);
   hObj = new TH1F("hObj", "Objective Function", 100, 0, 1000);
   hObjPos = new TH1F("hObjPos", "Objective Function Position", 50, 0, 1);
   hObjQ = new TH1F("hObjQ", "Objective Function Charge", 100, 0, 1000);

   hZvsObj = new TH2F("hZvsObj", "dZ vs Chi2", 20 + 1, -10, 10, 100, 0, 100);
   hZvsAmp = new TH2F("hZvsAmp", "dZ vs Amp", 20 + 1, -10, 10, 50, 0, 1);
   hAmpvsPosObj = new TH2F("hAmpvsPosObj", "Amp vs Pos Objective", 50, 0, 1, 50, 0, 0.5);
   hAmpvsObj = new TH2F("hAmpvsObj", "Amp vs Objective", 50, 0, 1, 50, 0, 100);
   hAmpvsLoc = new TH2F("hAmpvsLoc", "Amp vs Location", 50, 0, 1000, 50, 0, 1);
   FillPlots();
   if (draw)
      zHist->Draw();
}
