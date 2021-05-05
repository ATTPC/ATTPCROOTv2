void test_SpecMATMap()
{

   gSystem->Load("libAtTpcMap.so");
   AtMap *SpecMAT = new AtSpecMATMap();
   SpecMAT->GenerateAtTpc();
   SpecMAT->SetGUIMode();

   TH2Poly *padplane = SpecMAT->GetAtTpcPlane();

   for (auto i = 0; i < 10; ++i)
      padplane->Fill(i, -i, i * 100);

   padplane->Draw();
}
