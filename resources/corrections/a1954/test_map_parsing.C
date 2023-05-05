void test_map_parsing()
{

   TH2F *cMap = new TH2F("cMap", "cMap", 1000, 0, 1000, 275, 0, 275);

   AtSpaceChargeModel *model = new AtEDistortionModel();

   dynamic_cast<AtEDistortionModel *>(model)->SetCorrectionMaps("zLUT.txt", "radLUT.txt", "traLUT.txt");

   auto [zcorr, radcorr, tracorr] = dynamic_cast<AtEDistortionModel *>(model)->GetCorrectionFactors(200, 900);

   std::cout << " Correction factors : [" << zcorr << "," << radcorr << "," << tracorr << "]"
             << "\n";

   ROOT::Math::XYZPoint point(34.56, 100.1, 560.34); // Test point
   dynamic_cast<AtEDistortionModel *>(model)->CorrectSpaceCharge(point);

   for (Int_t i = 0; i < 1000; ++i)
      for (Int_t j = 0; j < 275; ++j) {
         auto [zcorr, radcorr, tracorr] = dynamic_cast<AtEDistortionModel *>(model)->GetCorrectionFactors(j, i);
         cMap->SetBinContent(i, j, radcorr);
      }

   cMap->Draw("zcol");
}
