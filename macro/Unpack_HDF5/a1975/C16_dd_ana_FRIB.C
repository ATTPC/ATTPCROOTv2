void C16_dd_ana_FRIB()
{
   FairRunAna *run = new FairRunAna(); // Forcing a dummy run

   std::vector<TString> files{"run_0011_FRIB.root"};

   for (auto iFile : files) {

      TFile *file = new TFile(iFile.Data(), "READ");

      TTree *tree = (TTree *)file->Get("cbmsim");
      Int_t nEvents = tree->GetEntries();
      std::cout << " Number of events : " << nEvents << std::endl;

      TTreeReader Reader1("cbmsim", file);
      TTreeReaderValue<TClonesArray> eventArray(Reader1, "AtRawEvent");

      for (Int_t i = 0; i < nEvents; i++) {

         std::cout << " Event Number : " << i << "\n";

         Reader1.Next();

         auto *rawEvent = (AtRawEvent *)eventArray->At(0);

         if (rawEvent) {

            auto genTraces = &rawEvent->GetGenTraces();

            if (auto trace = genTraces->at(0).get()) {
               auto rawADC = &trace->GetRawADC();
               for (auto i = 0; i < rawADC->size(); i++)
                  std::cout << i << " " << rawADC->at(i) << "\n";
            }
         }
      }
   }
}
