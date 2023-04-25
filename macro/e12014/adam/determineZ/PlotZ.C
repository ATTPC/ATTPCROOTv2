#include <TString.h>

TH1F *zHist = nullptr;
TH1F *aHist = nullptr;
TH1F *hAmp = nullptr;
TH2F *hBeam = nullptr;
void PlotZ()
{
   TString fileName = "/mnt/analysis/e12014/TPC/150Torr/cut1/Bi200.root";

   TChain tree("cbmsim");
   tree.Add(fileName);
   TTreeReader reader(&tree);
   TTreeReaderValue<TClonesArray> resultArray(reader, "AtMCResult");

   int zMin = 32;
   int zMax = 54;
   int aMin = zMin * (float)204 / 85;
   int aMax = zMax * (float)204 / 85;
   zHist = new TH1F("hZ", "Z", zMax - zMin, zMin, zMax);
   aHist = new TH1F("hA", "A", zMax - zMin, aMin, aMax);
   hBeam = new TH2F("hBeam", "Beam energy", 100, 0, 1000, 100, 0, 4500);
   hAmp = new TH1F("hAmp", "Charge Scaling Factor", 100, 0, 1);
   while (reader.Next()) {
      auto *result = dynamic_cast<MCFitter::AtMCResult *>(resultArray->At(0));
      if (result) {
         zHist->Fill(result->fParameters["Z0"]);
         zHist->Fill(result->fParameters["Z1"]);
         aHist->Fill(result->fParameters["A0"]);
         aHist->Fill(result->fParameters["A1"]);
         hBeam->Fill(result->fParameters["vZ"], result->fParameters["EBeam"]);
         hAmp->Fill(result->fParameters["Amp"]);
      }
   }

   zHist->Draw();
}
