void test_GADGETIIMap()
{

   gSystem->Load("libAtTpcMap.so");
   AtMap *gadget = new AtGadgetIIMap();
   gadget->GenerateAtTpc();
   gadget->SetGUIMode();

   TH2Poly *padplane = gadget->GetAtTpcPlane();

   for (auto i = 0; i < 20; ++i) {

      Int_t bin = padplane->Fill(i + 0.5, -i - 0.5, i * 100);
      std::cout << " Bin : " << bin << "\n";
   }

   gPad->Update();
}
