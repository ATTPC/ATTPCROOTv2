
void checkICLink(Int_t runNumber = 118)
{
   gSystem->Load("libAtReconstruction.so");

   TChain evtTr("E12014");
   evtTr.Add(TString::Format("./evtRun_%04d.root", runNumber));
   TChain tpcTr("cbmsim");
   tpcTr.Add(TString::Format("./run_%04d.root", runNumber));
   evtTr.AddFriend(&tpcTr);

   TTreeReader reader(&evtTr);
   TTreeReaderValue<HTMusicIC> ic(reader, "MUSIC");
   TTreeReaderValue<TClonesArray> eventArray(reader, "AtRawEvent");

   std::vector<double> icRatio;
   while (reader.Next()) {
      AtRawEvent *event = dynamic_cast<AtRawEvent *>(eventArray->At(0));

      // Search through and get the IC pad
      std::vector<Short_t> trace;
      for (auto &pad : *(event->GetPads()))
         if (pad.GetAuxName() == "IC") {
            for (int i = 0; i < 512; ++i)
               trace.push_back(pad.GetRawADC(i));
            break;
         }

      Short_t icPeak = *std::max_element(std::begin(trace), std::end(trace));
      icRatio.push_back(icPeak / ic->GetEnergy(0));
   }

   TGraph *gr = new TGraph(icRatio.size());
   for (int i = 0; i < icRatio.size(); ++i)
      gr->SetPoint(i, i, icRatio[i]);

   gr->Draw();
}

/*
   hash = HDFParserTask->CalculateHash(10, 0, 2, 34);
   HDFParserTask->SetAuxChannel(hash, "IC");
*/
