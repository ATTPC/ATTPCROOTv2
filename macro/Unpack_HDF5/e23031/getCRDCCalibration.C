/*
   Macro that carries out the calibration of the CRDC detectors of the S800. You will need to edit the filePathCRDC paths
   before running this macro.
   Input: + indexCRDC: Type Int_t. Either 1 or 2. It refers to which CRDC you want to calibrate.
*/

#include <FairLogger.h>

Double_t inchesToMm = 25.4; // mm/inch

Double_t slopeX = 2.54; // mm/pad

struct MaskHole {
   TString cutBranchName;

   TString holeID{""};
   Double_t xInches{-999};
   Double_t yInches{-999};

   Int_t nCounts{0};
   Double_t meanXCog{-999};
   Double_t stanDevXCog{-999};
   Double_t meanTac{-999};
   Double_t stanDevTac{-999};

   bool isProcessed{false};
};

Double_t CalibrateX(std::vector<MaskHole *> maskHoles, TFile *fileCRDC, TFile *cutFile, Int_t indexCRDC);
std::tuple<Double_t, Double_t> CalibrateY(std::vector<MaskHole *> maskHoles, TFile *fileCRDC, TFile *cutFile, Int_t indexCRDC);
std::tuple<Double_t, Double_t, Double_t, Double_t, Int_t> GetMeanXCogAndTAC(TFile *fileCRDC, TCutG *cut, Int_t indexCRDC);

void getCRDCCalibration(Int_t indexCRDC, TString cutFilePath = "")
{
   // 1->0 and 2->1 for convenience.
   indexCRDC--;

   // Histogram definitions.
   TH2F *maskHist = new TH2F("maskHist", "maskHist", 360, 50, 230, 2400, 600, 1800);
   TH1F *tacHist = new TH1F("tacHist", "tacHist", 2000, 0, 2000);
   TH1F *xCogHist = new TH1F("xCogHist", "xCogHist", 300, -0.5, 299.5);

   // Define the file path where the CRDC mask calibration run is.
   TString filePathCRDC{};
   if (indexCRDC == 0)
      filePathCRDC = "/media/aurio/Cris/11Li/dataS800/cal_juan/run-0125-00.root";
   else if (indexCRDC == 1)
      filePathCRDC = "/media/aurio/Cris/11Li/dataS800/cal_juan/run-0126-00.root";
   else {
      LOG(error) << "indexCRDC must be either 1 or 2!";
      return;
   }
   // Initialize FairRun and load the TTree of the CRDC calibration run.
   FairRunAna *run = new FairRunAna();

   LOG(info) << "Opening root file " << filePathCRDC.Data() << "...";
   TFile *fileCRDC = new TFile(filePathCRDC.Data(), "READ");
   if (fileCRDC->IsZombie()) {
      LOG(error) << "Could not open the file " << filePathCRDC.Data() << "!";
      return;
   }
   LOG(info) << "Done!";

   TTree *treeCRDC = (TTree *)fileCRDC->Get("caltree");
   Int_t nEntries = treeCRDC->GetEntries();
   LOG(info) << "The caltree TTree has a total of " << nEntries << " entries.";

   // First, we read the entire caltree and generate the xCOG and TAC histograms for the creation of TCuts of each visible hole of the mask.
   TTreeReader Reader("caltree", fileCRDC);
   TTreeReaderValue<S800Calc> *S800CalcArray = new TTreeReaderValue<S800Calc>(Reader, "s800calc");

   Double_t xCog{}, tac{};

   // Loop over all S800Calc entries.
   for (Int_t i = 0; i < nEntries; i++) {
      Reader.Next();
      S800Calc *S800Entry = (S800Calc *)S800CalcArray->Get();

      CRDC *CRDCEntry = S800Entry->GetCRDC(indexCRDC);
      xCog = CRDCEntry->GetXcog();
      tac = CRDCEntry->GetTAC();

      maskHist->Fill(xCog, tac);
      tacHist->Fill(tac);
      xCogHist->Fill(xCog);
   }

   // Draw the xCOG and TAC histograms.
   TCanvas *c = new TCanvas();
   maskHist->Draw("zcol");
   maskHist->GetXaxis()->SetTitle("x_{COG} (a.u.)");
   maskHist->GetYaxis()->SetTitle("TAC (a.u.)");

   TCanvas *c2 = new TCanvas();
   tacHist->Draw();
   tacHist->GetXaxis()->SetTitle("TAC (a.u.)");

   TCanvas *c3 = new TCanvas();
   xCogHist->Draw();
   xCogHist->GetXaxis()->SetTitle("x_{COG} (a.u.)");

   // If the TFile with the TCuts has been provided, we obtain the corresponding CRDC callibration.
   if (cutFilePath == "") {
      LOG(info) << "No TFile containing the mask hole cuts has been provided! Finishing now!";

      //Close the CRDC TFile.
      fileCRDC->Close();
      return;
   }

   LOG(info) << "Creating CRDC" << (indexCRDC + 1) << " callibration!";
   LOG(info) << "Opening root file " << cutFilePath.Data() << "...";
   TFile *cutFile = new TFile(cutFilePath.Data(), "READ");
   if (cutFile->IsZombie()) {
      LOG(error) << "Could not open the file " << cutFilePath.Data() << "!";

      //Close the CRDC TFile.
      fileCRDC->Close();
      return;
   }
   LOG(info) << "Done!";

   //Define a list of MaskHole objects to perform the callibration.
   MaskHole maskHole36;
   maskHole36.cutBranchName = "cutMaskHole36";
   maskHole36.holeID = "36";
   maskHole36.xInches = 12.3015;
   maskHole36.yInches = 6.9175;

   MaskHole maskHole37;
   maskHole37.cutBranchName = "cutMaskHole37";
   maskHole37.holeID = "37";
   maskHole37.xInches = 14.0742;
   maskHole37.yInches = 6.9175;

   MaskHole maskHole38;
   maskHole38.cutBranchName = "cutMaskHole38";
   maskHole38.holeID = "38";
   maskHole38.xInches = 14.4679;
   maskHole38.yInches = 6.9175;

   MaskHole maskHole39;
   maskHole39.cutBranchName = "cutMaskHole39";
   maskHole39.holeID = "39";
   maskHole39.xInches = 14.8616;
   maskHole39.yInches = 6.9175;

   MaskHole maskHole40;
   maskHole40.cutBranchName = "cutMaskHole40";
   maskHole40.holeID = "40";
   maskHole40.xInches = 15.2553;
   maskHole40.yInches = 6.9175;

   MaskHole maskHole59;
   maskHole59.cutBranchName = "cutMaskHole59";
   maskHole59.holeID = "59";
   maskHole59.xInches = 14.0742;
   maskHole59.yInches = 7.7049;

   MaskHole maskHole60;
   maskHole60.cutBranchName = "cutMaskHole60";
   maskHole60.holeID = "60";
   maskHole60.xInches = 14.0742;
   maskHole60.yInches = 7.3112;

   MaskHole maskStrip1;
   maskStrip1.cutBranchName = "cutMaskStrip1";
   maskStrip1.xInches = 10.794 + 0.063 / 2.;

   MaskHole maskStrip2;
   maskStrip2.cutBranchName = "cutMaskStrip2";
   maskStrip2.xInches = 16.700 + 0.063 / 2.;

   // We determine which mask holes we want to use for the x and y callibrations respectively.
   std::vector<MaskHole *> maskHolesForXCalibration;
   maskHolesForXCalibration.push_back(&maskHole36);
   maskHolesForXCalibration.push_back(&maskHole37);
   maskHolesForXCalibration.push_back(&maskHole38);
   maskHolesForXCalibration.push_back(&maskHole39);
   maskHolesForXCalibration.push_back(&maskHole40);
   maskHolesForXCalibration.push_back(&maskHole59);
   maskHolesForXCalibration.push_back(&maskHole60);
   maskHolesForXCalibration.push_back(&maskStrip1);
   maskHolesForXCalibration.push_back(&maskStrip2);

   std::vector<MaskHole *> maskHolesForYCalibration;
   maskHolesForYCalibration.push_back(&maskHole36);
   maskHolesForYCalibration.push_back(&maskHole37);
   maskHolesForYCalibration.push_back(&maskHole38);
   maskHolesForYCalibration.push_back(&maskHole39);
   maskHolesForYCalibration.push_back(&maskHole40);
   maskHolesForYCalibration.push_back(&maskHole59);
   maskHolesForYCalibration.push_back(&maskHole60);

   // Also, save the holes you want to mark in the final calibrated histogram in another vector.
   std::vector<MaskHole *> maskHolesForPlotting;
   maskHolesForPlotting.push_back(&maskHole36);
   maskHolesForPlotting.push_back(&maskHole37);
   maskHolesForPlotting.push_back(&maskHole38);
   maskHolesForPlotting.push_back(&maskHole39);
   maskHolesForPlotting.push_back(&maskHole40);
   maskHolesForPlotting.push_back(&maskHole59);
   maskHolesForPlotting.push_back(&maskHole60);

   // Now, we perform the calibrations.
   Double_t xOffset = CalibrateX(maskHolesForXCalibration, fileCRDC, cutFile, indexCRDC);
   auto [slopeY, yOffset] = CalibrateY(maskHolesForYCalibration, fileCRDC, cutFile, indexCRDC);

   // Finally, with the calibration parameters, we can plot the XY histogram to check if the mask is correct.
   TH2F *maskHistCalibrated = new TH2F("maskHistCalibrated", "maskHistCalibrated", 2 * 625, 0, 625, 2 * 352, 0, 352);

   TTreeReader Reader2("caltree", fileCRDC);
   TTreeReaderValue<S800Calc> *S800CalcArray2 = new TTreeReaderValue<S800Calc>(Reader2, "s800calc");

   for (Int_t i = 0; i < nEntries; i++) {
      Reader2.Next();
      S800Calc *S800Entry = (S800Calc *)S800CalcArray2->Get();

      CRDC *CRDCEntry = S800Entry->GetCRDC(indexCRDC);
      xCog = CRDCEntry->GetXcog();
      tac = CRDCEntry->GetTAC();

      Double_t x = slopeX * xCog + xOffset;
      Double_t y = slopeY * tac + yOffset;

      maskHistCalibrated->Fill(x, y);
   }

   TCanvas *c4 = new TCanvas();
   maskHistCalibrated->Draw("zcol");
   maskHistCalibrated->GetXaxis()->SetTitle("x (mm)");
   maskHistCalibrated->GetYaxis()->SetTitle("y (mm)");

   TGraph *holePositions = new TGraph(maskHolesForPlotting.size());
   Int_t i{0};
   TLatex *lt = new TLatex();
   for (MaskHole *hole : maskHolesForPlotting) {
      lt->DrawLatexNDC(hole->xInches / 24.603, hole->yInches / 13.835, hole->holeID);
      holePositions->SetPoint(i++, hole->xInches * inchesToMm, hole->yInches * inchesToMm);
   }
   holePositions->SetMarkerStyle(20);
   holePositions->SetMarkerSize(0.8);
   holePositions->Draw("P");

   // Close the CRDC TFile.
   fileCRDC->Close();

}

Double_t CalibrateX(std::vector<MaskHole *> maskHoles, TFile *fileCRDC, TFile *cutFile, Int_t indexCRDC)
{

   LOG(info) << "Calibrating X values of the CRDC" << (indexCRDC + 1) << ". Using the following mask holes:";

   Double_t xOffset{0};
   Int_t totalCounts{0};

   for (MaskHole *maskHole : maskHoles) {

      if (!maskHole->isProcessed) {
         TCutG *maskCut = (TCutG *)cutFile->Get(maskHole->cutBranchName);
         auto [meanXCog, stanDevXCog, meanTac, stanDevTac, nCounts] = GetMeanXCogAndTAC(fileCRDC, maskCut, indexCRDC);

         maskHole->nCounts = nCounts;
         maskHole->meanXCog = meanXCog;
         maskHole->stanDevXCog = stanDevXCog;
         maskHole->meanTac = meanTac;
         maskHole->stanDevTac = stanDevTac;
         maskHole->isProcessed = true;
      }

      Double_t xOffsetThisHole = maskHole->xInches * inchesToMm - slopeX * maskHole->meanXCog;

      LOG(info) << "Hole " << maskHole->cutBranchName << ":\n"
                           << "x       = " << maskHole->xInches << " inch,\n"
                           << "y       = " << maskHole->yInches << " inch,\n"
                           << "nCounts = " << maskHole->nCounts << ",\n"
                           << "xCOG    = " << maskHole->meanXCog << " pad,\n"
                           << "TAC     = " << maskHole->meanTac << " a.u.,\n"
                           << "                  => xOffset = " << xOffsetThisHole << " mm.\n";

      // Add this xOffset to the mean.
      xOffset += maskHole->nCounts * xOffsetThisHole;
      totalCounts += maskHole->nCounts;
   }

   xOffset /= totalCounts;
   LOG(info) << "Final xOffset obtained for the CRDC" << (indexCRDC + 1) << ": xOffset = " << xOffset << "mm.";

   return xOffset;
}

std::tuple<Double_t, Double_t> CalibrateY(std::vector<MaskHole *> maskHoles, TFile *fileCRDC, TFile *cutFile, Int_t indexCRDC)
{

   LOG(info) << "Calibrating Y values of the CRDC" << (indexCRDC + 1) << ". Using the following mask holes:";

   TGraph *fitPoints = new TGraph(maskHoles.size());
   Int_t i{0};

   for (MaskHole *maskHole : maskHoles) {

      if (!maskHole->isProcessed) {
         TCutG *maskCut = (TCutG *)cutFile->Get(maskHole->cutBranchName);
         auto [meanXCog, stanDevXCog, meanTac, stanDevTac, nCounts] = GetMeanXCogAndTAC(fileCRDC, maskCut, indexCRDC);

         maskHole->nCounts = nCounts;
         maskHole->meanXCog = meanXCog;
         maskHole->stanDevTac = stanDevXCog;
         maskHole->meanTac = meanTac;
         maskHole->stanDevTac = stanDevTac;
         maskHole->isProcessed = true;
      }

      fitPoints->SetPoint(i++, maskHole->meanTac, maskHole->yInches * inchesToMm);

      LOG(info) << "Hole " << maskHole->cutBranchName << ":\n"
                           << "x       = " << maskHole->xInches << " inch,\n"
                           << "y       = " << maskHole->yInches << " inch,\n"
                           << "nCounts = " << maskHole->nCounts << ",\n"
                           << "xCOG    = " << maskHole->meanXCog << " pad,\n"
                           << "TAC     = " << maskHole->meanTac << " a.u.\n";
   }

   TF1 *fitLine = new TF1("fitLine", "[0]*x+[1]", 0, 2000);

   TFitResultPtr fitResult = fitPoints->Fit(fitLine);

   LOG(info) << "Final slopeY and yOffset obtained for the CRDC" << (indexCRDC + 1) << ":\n"
             << "                  => slopeY  = " << fitLine->GetParameter(0) << " mm/a.u.,\n"
             << "                  => yOffset = " << fitLine->GetParameter(1) << " mm.";

   return std::make_tuple(fitLine->GetParameter(0), fitLine->GetParameter(1));
}

std::tuple<Double_t, Double_t, Double_t, Double_t, Int_t> GetMeanXCogAndTAC(TFile *fileCRDC, TCutG *cut, Int_t indexCRDC)
{
   TTree *treeCRDC = (TTree *)fileCRDC->Get("caltree");
   Int_t nEntries = treeCRDC->GetEntries();

   TTreeReader Reader("caltree", fileCRDC);
   TTreeReaderValue<S800Calc> *S800CalcArray = new TTreeReaderValue<S800Calc>(Reader, "s800calc");

   Double_t xCog{}, tac{};
   Double_t meanXCog{}, meanTac{};
   Double_t meanXCog2{}, meanTac2{};
   Int_t nEntriesInCut{};

   // Loop over all S800Calc entries.
   for (Int_t i = 0; i < nEntries; i++) {
      Reader.Next();
      S800Calc *S800Entry = (S800Calc *)S800CalcArray->Get();

      CRDC *CRDCEntry = S800Entry->GetCRDC(indexCRDC);
      xCog = CRDCEntry->GetXcog();
      tac = CRDCEntry->GetTAC();
      if (cut->IsInside(xCog, tac)) {
         meanXCog += xCog;
         meanTac += tac;

         meanXCog2 += std::pow(xCog, 2);
         meanTac2 += std::pow(tac, 2);

         nEntriesInCut++;
      }
   }
   meanXCog /= nEntriesInCut;
   meanTac /= nEntriesInCut;

   meanXCog2 /= nEntriesInCut;
   meanTac2 /= nEntriesInCut;

   Double_t stanDevXCog = std::sqrt(meanXCog2 - std::pow(meanXCog, 2));
   Double_t stanDevTac = std::sqrt(meanTac2 - std::pow(meanTac, 2));

   return std::make_tuple(meanXCog, stanDevXCog, meanTac, stanDevTac, nEntriesInCut);
}
