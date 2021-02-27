void test_GADGETIIMap(){

	gSystem->Load("libAtTpcMap.so");
	AtMap *gadget = new AtGadgetIIMap();
	gadget->GenerateATTPC();
	gadget->SetGUIMode();
        
	TH2Poly *padplane = gadget->GetATTPCPlane();

	for(auto i=0;i<10;++i)
	  padplane->Fill(i,-i,i*100);

        gPad->Update();

}


