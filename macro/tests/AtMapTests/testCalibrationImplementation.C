/*
   Macro to test if the implementation of the calibration mapping works as intended.
*/

double polyOrder3(double *x, double *params);
double exponential(double *x, double *params);

void testCalibrationImplementation()
{
   // Define an AtTpcMap to test the polynomial function.
   AtTpcMap *mapPoly = new AtTpcMap();
   mapPoly->SetCalibrationFunction(polyOrder3);
   mapPoly->ParseCalibrationParameters("./testCalibrationParametersPoly.csv", 4);

   double ADC_value = -20;
   int numPoints = 40;
   double stepSize = 40./numPoints;

   TGraph *graphPolyPad1 = new TGraph(numPoints);
   TGraph *graphPolyPad2 = new TGraph(numPoints);
   TGraph *graphPolyPad3 = new TGraph(numPoints);
   for (int i = 0; i < numPoints; i++) {
      graphPolyPad1->SetPoint(i, ADC_value, mapPoly->GetCalibratedELoss(1, ADC_value));
      graphPolyPad2->SetPoint(i, ADC_value, mapPoly->GetCalibratedELoss(2, ADC_value));
      graphPolyPad3->SetPoint(i, ADC_value, mapPoly->GetCalibratedELoss(3, ADC_value));

      ADC_value += stepSize;
   }

   TF1 *actualPolyPad1 = new TF1("actualPolyPad1", polyOrder3, -20, 20, 4);
   actualPolyPad1->SetParameters(2.5, -3, -0.05, 0.01); // Directly from the .csv file.

   TF1 *actualPolyPad2 = new TF1("actualPolyPad2", polyOrder3, -20, 20, 4);
   actualPolyPad2->SetParameters(-3, 5, 0.01, -0.005); // Directly from the .csv file (ignoring last extra entry).

   TH1F *dummyHist = new TH1F("dummyHist", "dummyHist", 10, -20, 20);
   TCanvas *cPoly = new TCanvas();
   cPoly->Divide(3, 1);
   cPoly->cd(1);
   dummyHist->Draw();
   actualPolyPad1->Draw("same");
   graphPolyPad1->Draw("same*");
   dummyHist->SetTitle("Pad 1");
   dummyHist->GetXaxis()->SetTitle("ADC");
   dummyHist->GetYaxis()->SetTitle("Calibration function");
   dummyHist->GetYaxis()->SetRangeUser(-30, 30);

   cPoly->cd(2);
   TH1F *clone1 = (TH1F *)dummyHist->DrawClone();
   actualPolyPad2->Draw("same");
   graphPolyPad2->Draw("same*");
   clone1->SetTitle("Pad 2");
   clone1->GetXaxis()->SetTitle("ADC");
   clone1->GetYaxis()->SetTitle("Calibration function");
   clone1->GetYaxis()->SetRangeUser(-80, 80);

   cPoly->cd(3);
   TH1F *clone2 = (TH1F *)dummyHist->DrawClone();
   graphPolyPad3->Draw("same*");
   clone2->SetTitle("Pad 3");
   clone2->GetXaxis()->SetTitle("ADC");
   clone2->GetYaxis()->SetTitle("Calibration function");
   clone2->GetYaxis()->SetRangeUser(-1200, 0);

   // Define another AtTpcMap to test the exponential function.
   AtTpcMap *mapExp = new AtTpcMap();
   mapExp->SetCalibrationFunction(exponential);
   mapExp->ParseCalibrationParameters("./testCalibrationParametersExponential.csv", 2);

   ADC_value = -20;

   TGraph *graphExpPad1 = new TGraph(numPoints);
   TGraph *graphExpPad2 = new TGraph(numPoints);
   TGraph *graphExpPad3 = new TGraph(numPoints);
   for (int i = 0; i < numPoints; i++) {
      graphExpPad1->SetPoint(i, ADC_value, mapExp->GetCalibratedELoss(1, ADC_value));
      graphExpPad2->SetPoint(i, ADC_value, mapExp->GetCalibratedELoss(2, ADC_value));
      graphExpPad3->SetPoint(i, ADC_value, mapExp->GetCalibratedELoss(3, ADC_value));

      ADC_value += stepSize;
   }

   TF1 *actualExpPad1 = new TF1("actualExpPad1", exponential, -20, 20, 2);
   actualExpPad1->SetParameters(10,0.1); // Directly from the .csv file.

   TF1 *actualExpPad2 = new TF1("actualExpPad2", exponential, -20, 20, 2);
   actualExpPad2->SetParameters(20,-0.05); // Directly from the .csv file (ignoring last extra entry).

   TCanvas *cExp = new TCanvas();
   cExp->Divide(3, 1);
   cExp->cd(1);
   TH1F *clone3 = (TH1F *)dummyHist->DrawClone();
   actualExpPad1->Draw("same");
   graphExpPad1->Draw("same*");
   clone3->SetTitle("Pad 1");
   clone3->GetXaxis()->SetTitle("ADC");
   clone3->GetYaxis()->SetTitle("Calibration function");
   clone3->GetYaxis()->SetRangeUser(0, 80);

   cExp->cd(2);
   TH1F *clone4 = (TH1F *)dummyHist->DrawClone();
   actualExpPad2->Draw("same");
   graphExpPad2->Draw("same*");
   clone4->SetTitle("Pad 2");
   clone4->GetXaxis()->SetTitle("ADC");
   clone4->GetYaxis()->SetTitle("Calibration function");
   clone4->GetYaxis()->SetRangeUser(0, 80);

   cExp->cd(3);
   TH1F *clone5 = (TH1F *)dummyHist->DrawClone();
   graphExpPad3->Draw("same*");
   clone5->SetTitle("Pad 3");
   clone5->GetXaxis()->SetTitle("ADC");
   clone5->GetYaxis()->SetTitle("Calibration function");
   clone5->GetYaxis()->SetRangeUser(-1200, 0);

   // Check what happens when asked about a pad that does not exist in the .csv files.
   mapPoly->GetCalibratedELoss(4, ADC_value);
   mapExp->GetCalibratedELoss(4, ADC_value);
}

double polyOrder3(double *x, double *params)
{
   double xx = *x;
   return params[0] + params[1] * xx + params[2] * std::pow(xx, 2) + params[3] * std::pow(xx, 3);
}

double exponential(double *x, double *params)
{
   double xx = *x;
   return params[0] * std::exp(params[1] * xx);
}
