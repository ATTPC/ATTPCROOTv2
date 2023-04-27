#include <TString.h>

TH1F *zHist = nullptr;
TH1F *aHist = nullptr;
TH1F *hAmp = nullptr;
TH2F *hBeam = nullptr;
TH1F *hObj = nullptr;
TChain *tree;

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

void FillPlots(float EMin = 0, float EMax = 5000)
{

   TTreeReader reader(tree);
   TTreeReaderValue<TClonesArray> resultArray(reader, "AtMCResult");
   zHist->Reset();
   aHist->Reset();
   hAmp->Reset();
   hObj->Reset();
   hBeam->Reset();

   while (reader.Next()) {
      auto *result = dynamic_cast<MCFitter::AtMCResult *>(resultArray->At(0));
      if (result) {
         if (result->fParameters["vZ"] > EMin && result->fParameters["vZ"] < EMax) {
            zHist->Fill(result->fParameters["Z0"]);
            zHist->Fill(result->fParameters["Z1"]);
            // zHist->Fill(GetAvg("Z0", *resultArray));
            // zHist->Fill(GetAvg("Z1", *resultArray));

            aHist->Fill(result->fParameters["A0"]);
            aHist->Fill(result->fParameters["A1"]);
            hAmp->Fill(result->fParameters["Amp"]);
            hObj->Fill(result->fObjective);
         }
         hBeam->Fill(result->fParameters["vZ"], result->fParameters["EBeam"]);
      }
   }

   zHist->Draw();
}

void PlotZ()
{
   TString fileName = "/mnt/analysis/e12014/TPC/150Torr_nomod/cut1/SRIM/Bi200.root";

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

   FillPlots();
}
