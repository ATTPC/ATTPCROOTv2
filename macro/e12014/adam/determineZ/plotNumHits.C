#include "TH1F.h"
#include "TRandom3.h"

#include <iostream>
#include <thread>
#include <vector>

int plotNumHits()
{
   TChain tree("cbmsim");
   tree.Add("/mnt/analysis/e12014/TPC/150Torr_yFit/Bi200.root");
   E12014::CreateMap();

   TH1F *hits = new TH1F("hH", "hits", 100, 0, 1000);

   TTreeReader reader(&tree);
   TTreeReaderValue<TClonesArray> fissionArray(reader, "AtFissionEvent");
   while (reader.Next()) {
      if (reader.GetCurrentEntry() % 1000 == 0)
         cout << reader.GetCurrentEntry() << endl;

      AtFissionEvent *event = dynamic_cast<AtFissionEvent *>(fissionArray->At(0));
      if (event == nullptr)
         cout << "Could not find event";

      hits->Fill(event->GetBeamHits().size() + event->GetFragHits().size());

      if (event->GetEventID() == 10188) {
         int numBad = 0;
         for (const auto hit : event->GetFragHits()) {
            auto padRef = E12014::fMap->GetPadRef(hit->GetPadNum());
            if (padRef.cobo == 3 && padRef.asad == 2)
               numBad++;
         }
         std::cout << numBad << " in event " << event->GetEventID() << " " << reader.GetCurrentEntry() << std::endl;
      }
   }
   hits->Draw();
   return 0;
}
