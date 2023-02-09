void ADCsumPlot(AtTabInfo *tabInfo) {
    cout << "starting ADCsumPlot" << endl;
    auto rawEvent = (tabInfo->GetAugment<AtTabInfoFairRoot<AtRawEvent>>("AtRawEvent"))->GetInfo();
    //auto rawEvent = (tabInfo->GetAugment<AtTabInfoFairRoot>("AtRawEvent"))->GetInfo();

   if (rawEvent == nullptr) {
      LOG(debug) << "fRawEvent is nullptr for ADCsumPlot! Please set the raw event branch.";
      return;
   }

    string plotName = "adcSumPlot";

    if(!gROOT->GetListOfSpecials()->FindObject(plotName.c_str()))
    {
        TH1D *adcSumPlot = new TH1D(plotName.c_str(), plotName.c_str(), 512, 0, 512);
        gROOT->GetListOfSpecials()->Add(adcSumPlot);
    }

    auto plot = dynamic_cast<TH1D *>(gROOT->GetListOfSpecials()->FindObject(plotName.c_str()));
    for(int i = 0; i < 513; i++) {
        plot->SetBinContent(i, 0);
    }

    auto tpcMap = AtViewerManager::Instance()->GetMap();
 
    for(auto it = rawEvent->GetPads().begin(); it != rawEvent->GetPads().end(); it++) {
        auto padCent = tpcMap->CalcPadCenter((*it)->GetPadNum());
        //if(sqrt(padCent.X() * padCent.X() + padCent.Y() * padCent.Y()) < 50 && (tpcMap->GetPadRef((*it)->GetPadNum())).cobo != 2 && (tpcMap->GetPadRef((*it)->GetPadNum())).cobo != 6) {
        if(sqrt(padCent.X() * padCent.X() + padCent.Y() * padCent.Y()) < 50) {
            auto pad = rawEvent->GetPad((*it)->GetPadNum());
            if(dynamic_cast<AtPadArray *>(pad->GetAugment("Qreco")) == nullptr)
                return;
            double maxAdc = 0;
            for(int i = 0; i < 512; i++) {
                if(pad->GetADC(i) > maxAdc) {
                    maxAdc = pad->GetADC(i);
                }
            }
            if(maxAdc < 400) { 
                for(int i = 0; i < 512; i++) {
                    plot->AddBinContent(i + 1, pad->GetADC(i));
                    //plot->AddBinContent(i + 1, dynamic_cast<AtPadArray *>(pad->GetAugment("Qreco"))->GetArray(i));
                }
            }
        }
    }

    plot->Draw();

    cout << "ADC summed" << endl;
}