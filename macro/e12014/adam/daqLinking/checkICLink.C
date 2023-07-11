#include <FairLogger.h>
TGraph *gr = nullptr;
TH2F *hNSCLvsTPC = new TH2F("hNSCLvsTPC", "MUSIC Correlation", 100, 500, 2000, 100, 0.5, 1);
TH2F *hNSCLvsTPCCorrupt = new TH2F("hNSCLvsTPCCorr", "MUSIC Correlation", 100, 500, 2000, 100, 0.5, 1);

// Draw correlation plots between IC signal in HiRAEVT and TPC DAQs
// Takes in TPC run number
void checkICLink(Int_t runNumber = 130)
{
   LOG(info) << "Processing event";

   gSystem->Load("libAtReconstruction.so");

   TChain evtTr("E12014");
   evtTr.Add(TString::Format("/mnt/analysis/e12014/TPC/fission_linked_nomod/evtRun_%04d.root", runNumber));
   TChain tpcTr("cbmsim");
   tpcTr.Add(TString::Format("/mnt/analysis/e12014/TPC/fission_linked_nomod/run_%04d.root", runNumber));
   evtTr.AddFriend(&tpcTr);

   TTreeReader reader(&evtTr);
   TTreeReaderValue<HTMusicIC> ic(reader, "MUSIC");
   TTreeReaderValue<TClonesArray> eventArray(reader, "AtRawEventRaw");

   std::vector<double> icRatio;
   double oldIC = -1;
   while (reader.Next()) {

      if (reader.GetCurrentEntry() == 10000)
         break;

      if (reader.GetCurrentEntry() % 100 == 0)
         LOG(info) << "Processing event: " << reader.GetCurrentEntry();

      AtRawEvent *event = dynamic_cast<AtRawEvent *>(eventArray->At(0));

      auto icPad = event->GetAuxPad("IC");
      if (icPad == nullptr)
         continue;

      Short_t icPeak = *std::max_element(std::begin(icPad->GetRawADC()), std::end(icPad->GetRawADC()));
      icRatio.push_back(icPeak / ic->GetEnergy(0));

      // hNSCLvsTPC->Fill(icPeak, oldIC);
      if (reader.GetCurrentEntry() >= 136 && reader.GetCurrentEntry() < 329)
         hNSCLvsTPCCorrupt->Fill(icPeak, ic->GetEnergy(0));
      else
         hNSCLvsTPC->Fill(icPeak, ic->GetEnergy(0));
      oldIC = ic->GetEnergy(0);
   }

   LOG(info) << "Done reading tree";

   gr = new TGraph(icRatio.size());
   for (int i = 0; i < icRatio.size(); ++i)
      gr->SetPoint(i, i, icRatio[i]);

   hNSCLvsTPC->Draw("colz");
}

/*
   hash = HDFParserTask->CalculateHash(10, 0, 2, 34);
   HDFParserTask->SetAuxChannel(hash, "IC");
*/
