void DrawAuxPad(AtTabInfo *tabInfo, string name)
{
   cout << "starting DrawAuxPad" << endl;
   auto rawEvent = (tabInfo->GetAugment<AtTabInfoFairRoot<AtRawEvent>>("AtRawEvent"))->GetInfo();
   // auto rawEvent = dynamic_cast<AtTabInfoFairRoot<AtRawEvent> *>(tabInfo->GetAugment("AtRawEvent"))->GetInfo();

   if (rawEvent == nullptr) {
      LOG(debug) << "fRawEvent is nullptr for DrawAuxPad! Please set the raw event branch.";
      return;
   }

   string plotName = name + "Plot";

   if (!gROOT->GetListOfSpecials()->FindObject(plotName.c_str())) {
      TH1D *histPlot = new TH1D(plotName.c_str(), plotName.c_str(), 512, 0, 512);
      gROOT->GetListOfSpecials()->Add(histPlot);
   }

   auto plot = dynamic_cast<TH1D *>(gROOT->GetListOfSpecials()->FindObject(plotName.c_str()));
   auto auxPad = rawEvent->GetAuxPad(name);
   if (auxPad != nullptr) {
      for (int i = 0; i < 513; i++) {
         plot->SetBinContent(i, auxPad->GetADC(i));
      }
      plot->Draw();
      cout << name + " aux pad drawn" << endl;
   } else
      cout << "Aux Pad is null" << endl;
}