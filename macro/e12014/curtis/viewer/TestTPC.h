bool TestTPC(AtTabInfoFairRoot<AtRawEvent> *rawEventInfo) {
   auto rawEvent = rawEventInfo->GetInfo();

   if (rawEvent == nullptr) {
      LOG(debug) << "event is nullptr for TestTPC! Please set the event branch.";
      return false;
   }

    auto tpcMap = AtViewerManager::Instance()->GetMap();

    int numHits = 0;
 
    for(auto it = rawEvent->GetPads().begin(); it != rawEvent->GetPads().end(); it++) {
        auto padCent = tpcMap->CalcPadCenter((*it)->GetPadNum());
        //if(sqrt(padCent.X() * padCent.X() + padCent.Y() * padCent.Y()) < 50 && (tpcMap->GetPadRef((*it)->GetPadNum())).cobo != 2 && (tpcMap->GetPadRef((*it)->GetPadNum())).cobo != 6) {
        if(sqrt(padCent.X() * padCent.X() + padCent.Y() * padCent.Y()) < 50) {
            auto pad = rawEvent->GetPad((*it)->GetPadNum());
            double maxAdc = 0;
            for(int i = 0; i < 512; i++) {
                if(pad->GetADC(i) > maxAdc) {
                    maxAdc = pad->GetADC(i);
                }
            }
            if(maxAdc > 40) { 
                numHits++;
            }
        }
    }

   return numHits > 5;
}