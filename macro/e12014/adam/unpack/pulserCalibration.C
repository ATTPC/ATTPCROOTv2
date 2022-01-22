#include "../helper.h"
#include "../AtGenerators/AtCSVReader.h"
#include "TString.h"
#include "TFile.h"
#include "TFitResultPtr.h"
#include "TGraph.h"
#include <map>
#include <fstream>

TString filePath = "/mnt/analysis/e12014/TPC/unpacked/run_%04d.root";
using voltChPair = std::pair<double, double>;

/***** "public" funtions ******/
/* Creates two files
 * output/run{RunNum}.root: histogram of the max of each pad
 * output/run{RunNum}.csv: csv of padID and mean for each pad
 */
void getMaxOfAllPads(int runNum);
void calibrateAllPads(std::map<int, double> &runAndVoltage);
void calibrateAllPads();

/******* "private" variables ******/
//Used for getting ch value for a single pulser run
std::map<int, TH1F*> padHist;
std::map<int, TH2F*> corrHist;
std::vector<std::pair<int, double>> maxPeak;
//Used for getting calibration from multiple runs
std::map<int, std::vector<voltChPair>> runPadMean; //[chNum] -> {voltage, ch}
std::map<int, std::vector<double>> fitParameters;
std::map<int, TGraph*> fitGraphMap;

/******* "private" functions ******/
void fillHistograms();
void fitHistograms();
TFitResultPtr& fitHistogram(TH1F* hist, int max, int range);
void writeResults(int runNum);
void fillPad(int padNum, double max);
void fillCorr(int padNum, double max, double comp);
void clear();
void loadFittedData(const std::map<int, double> &runAndVoltage);
void fillGraphs();
void fitGraphs();
void saveCalibration();

/* Creates two files
 * run{RunNum}.root: histogram of the max of each pad
 * run{RunNum}.csv: csv of padID and mean for each pad
 */
void getMaxOfAllPads(int runNum)
{
   clear();
   
   // Load the run
   loadRun(TString::Format(filePath, runNum), "AtRawEvent", "AtEventH");
   
   fillHistograms();
   fitHistograms();
   writeResults(runNum);
}

void calibrateAllPads(std::map<int, double> &runAndVoltage)
{
   clear();
   loadFittedData(runAndVoltage);
   fillGraphs();
   fitGraphs();
   saveCalibration();
}
void calibrateAllPads()
{
   std::map<int, double> rV;
   rV.insert({193, 1000});
   rV.insert({195, 500});
   rV.insert({196, 50});
   rV.insert({197, 100});
   calibrateAllPads(rV);
}
void fillPad(int padNum, double max)
{
   auto hist = padHist.find(padNum);
   if(padHist.find(padNum) == padHist.end())
   {
      //std::cout << "Creating new hist for pad: " << padNum << std::endl;
      auto newHist = new TH1F(TString::Format("pad_%d", padNum),TString::Format("Pad %d", padNum),
			      4096/2, 0, 4096);
      padHist.insert({padNum, newHist});
   }

   padHist[padNum]->Fill(max);
}

void fillCorr(int padNum, double max, double comp)
{
   auto hist = corrHist.find(padNum);
   if(hist == corrHist.end())
   {
      //std::cout << "Creating new hist for pad: " << padNum << std::endl;
      auto newHist = new TH2F(TString::Format("padCorr_%d", padNum),TString::Format("Pad %d", padNum),
			      300, 0, 4096, 300, 0, 4096);
      newHist->Fill(comp, max);
      corrHist.insert({padNum, newHist});
   }
   else
      hist->second->Fill(comp, max);
}

void fillHistograms()
{
   while(nextEvent())
   {
      std::cout << "Looking at event: " << reader->GetCurrentEntry() << std::endl;
      
      for(const auto &hit : *(eventPtr->GetHitArray()))
      	 fillPad(hit.GetHitPadNum(), hit.GetCharge());
      
   }

}

void writeResults(int runNum)
{
   //Create output file
   TFile *oFile = new TFile(TString::Format("output/run_%d.root", runNum), "RECREATE");
   for(auto &pair : padHist)
      pair.second->Write();
   for(auto &pair : corrHist)
      pair.second->Write();
   oFile->Close();

   //Write csv
   std::ofstream csv(TString::Format("output/run_%d.csv", runNum));
   for(auto &pair : maxPeak)
      csv << pair.first << "," << pair.second << std::endl;
   csv.close();
}

void fitHistograms()
{
   std::cout << "Fitting histograms" << std::endl;
   for(auto &pair : padHist)
   {
      auto hist = pair.second;
      //hist->Rebin(2);
      auto maxBin = hist->GetMaximumBin();
      auto max = hist->GetBinCenter(maxBin);
      auto widthMin = 10;
      auto widthMax = 10;

      auto fit = hist->Fit("gaus", "SQ", "", max-widthMin, max+widthMax);

      //Only save if we are in the linear region of electronics
      if(fit->Parameter(1) < 3500)
	 maxPeak.emplace_back(std::pair<int,double>(pair.first, fit->Parameter(1)));

   }
   std::cout << "Done fitting histograms" << std::endl;
}

void clear()
{
    for(auto &pair : padHist)
       delete pair.second;
   for(auto &pair : corrHist)
      delete pair.second;
   for(auto &pair : fitGraphMap)
      delete pair.second;
   
   padHist.clear();
   corrHist.clear();
   maxPeak.clear();
   runPadMean.clear();
   fitParameters.clear();
   fitGraphMap.clear();
}

void loadFittedData(const std::map<int, double> &runAndVoltage)
{
   for(auto &pair : runAndVoltage)
   {
      auto volt = pair.second;
      std::ifstream csv(TString::Format("output/run_%d.csv", pair.first));

      //CSVRange from AtCSVReader.h
      for(auto &row : CSVRange<double>(csv))
      {
	 int padNum = row[0];
	 double ch = row[1];
	 if(runPadMean.find(padNum) == runPadMean.end())
	    runPadMean.insert({padNum, std::vector<voltChPair>()});
	 runPadMean[padNum].emplace_back(voltChPair(volt, ch));
      }
   }
}

void fillGraphs()
{
   for(auto &padPair : runPadMean)
   {

      int padNum = padPair.first;
      if(padPair.second.size() < 3)
      {
	 std::cout << "Pad " << padNum << " has only " << padPair.second.size()
		   << " valid points. Skipping fitting." << std::endl;
	 continue;
      }

      auto graphIt = fitGraphMap.find(padNum);
      if(graphIt == fitGraphMap.end())
	 fitGraphMap.insert({padNum, new TGraph(padPair.second.size())});

      int i = 0;
      for(auto &voltCh : padPair.second)
	 fitGraphMap[padNum]->SetPoint(i++, voltCh.second, voltCh.first);

   }
}
void fitGraphs()
{
   for(auto &graphPair : fitGraphMap)
   {
      auto padNum = graphPair.first;
      auto graph = graphPair.second;
      auto fit = graph->Fit("pol1", "SQ");
      if((Int_t)fit != 0)
      {
	 std::cout << "Failed to fit pad: " << padNum << std::endl;
	 continue;
      }
      
      fitParameters.insert({graphPair.first, {}});
      fitParameters[padNum].emplace_back(fit->Parameter(0));
      fitParameters[padNum].emplace_back(fit->Parameter(1));
   }
}
void saveCalibration()
{
   std::ofstream csv("output/calibration.csv");
   TH1F gainHist("gainHist", "Pad Gains", 1000, 0, 1);
   TH1F pedestalHist("pedestalHist", "Pad Pedestals", 200, -100, 100);

   for(auto &pair : fitParameters)
   {
      csv << pair.first << "," << pair.second[0] << "," << pair.second[1] << std::endl;
      gainHist.Fill(pair.second[1]);
      pedestalHist.Fill(pair.second[0]);
   }
   csv.close();
   
   //Create output file
   TFile *oFile = new TFile("output/calibration.root", "RECREATE");
   oFile->cd();
   for(auto &pair : fitGraphMap)
      pair.second->Write(TString::Format("pad_%d", pair.first));
   gainHist.Write();
   pedestalHist.Write();
   oFile->Close();


}
