
#include <TChain.h>
#include <TH1.h>
#include <TH2.h>
#include <TString.h>
#include <TSystem.h>
#include <TTreeReader.h>

std::vector<TH2F *> fHist;
void checkTest()

{
   TChain chain("cbmsim");
   chain.Add("./data/fitterTest.root");
   TTreeReader reader(&chain);

   TTreeReaderValue<TClonesArray> array(reader, "AtMCResult");
   // TTreeReaderValue<TClonesArray> arrayFission(reader, "AtFissionEvent");
   int i = 0;
   while (reader.Next()) {
      i++;

      /*auto fissionEvent = dynamic_cast<AtFissionEvent *>(arrayFission->At(0));
      if (!fissionEvent) {
         std::cout << "fuck";
         continue;
      }
      */
      double Z = dynamic_cast<MCFitter::AtMCResult *>(array->At(0))->fParameters["vZ"];
      fHist.push_back(new TH2F(TString::Format("h%d", i), TString::Format("h%f", Z), 100, 0, 200, 100, 2000, 6000));

      for (int i = 0; i < array->GetEntries(); ++i) {
         auto res = dynamic_cast<MCFitter::AtMCResult *>(array->At(i));
         fHist.back()->Fill(res->fObjective, res->fParameters["EBeam"]);
      }
   }
}
