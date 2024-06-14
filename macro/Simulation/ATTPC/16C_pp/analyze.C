void analyze()
{

   FairRunAna *run = new FairRunAna();

   std::ofstream hitsFile;
   hitsFile.open("hits.txt");

   TFile *file = new TFile("./data/output_digi.root", "READ");
   TTree *tree = (TTree *)file->Get("cbmsim");
   Int_t nEvents = tree->GetEntries();
   std::cout << " Number of events : " << nEvents << std::endl;

   TTreeReader Reader("cbmsim", file);
   TTreeReaderValue<TClonesArray> eventHArray(Reader, "AtEventH");

   for (Int_t i = 0; i < nEvents; i++) {

      Reader.Next();

      AtEvent *event = (AtEvent *)eventHArray->At(0);

      if (event) {

         auto &hitArray = event->GetHits();
         std::cout << " Number of hits : " << hitArray.size() << std::endl;

         for (auto &hit : hitArray) {
            auto pos = hit->GetPosition();
            auto charge = hit->GetCharge();
            auto time = hit->GetTimeStamp();
            hitsFile << pos.X() << " " << pos.Y() << " " << pos.Z() << "  " << time << "  " << charge << std::endl;
         }
      }
   }

   hitsFile.close();
   file->Close();
}