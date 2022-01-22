/*
 * Code for testing a digital trapezoid filter.
 * Filter from: "Digital techniques for real-time pulse shaping in radiation measurements"
 *
 */
#include "TString.h"
#include "TH1.h"
#include "TCanvas.h"
#include "TFrame.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

using vecFloat = std::vector<float>;

int k, l;
float M;
float d(const vecFloat &signal, int n);
float p(const vecFloat &signal, int n);
float r(const vecFloat &signal, int n);
float s(const vecFloat &signal, int n);

vecFloat *readFile(std::ifstream &file);
TH1F *createHistogram(const vecFloat &data, TString name);
int removeBaselineAndGetStart(vecFloat *data, int threshold);
void rescaleWaveform(vecFloat *data, int start, float scale);
int getMax(const vecFloat &data, int start);

void testFilter(float MIn = 17.5, int kIn = 10, int lIn = 30, TString waveformFile = "waveform.dat")
{
   k = kIn;
   l = lIn;
   M = MIn;

   // Exponential with start t_o and decay constant Tau
   // y(t) = y_o * exp(-(t-t_0)/Tau) if t > t_0
   //      = 0 if t < t_0

   // Open the waveform file and create a histogram
   std::ifstream waveFile;
   waveFile.open(waveformFile.Data());
   if (!waveFile.is_open())
      return;

   std::cout << "Opened file: " << waveformFile;

   auto waveform = readFile(waveFile);
   auto start = removeBaselineAndGetStart(waveform, 100);
   
   
   std::cout << "Start at tb: " << start << std::endl;

   vecFloat shiftedWaveform;
   for (int i = start; i < waveform->size(); ++i)
      shiftedWaveform.push_back(waveform->at(i));

   std::cout << "Shifted waveform" << std::endl;

   
   vecFloat filtered;
   for (int i = 0; i < waveform->size(); ++i)
   {
      if (i < start)
         filtered.push_back(0);
      else
	 filtered.push_back(s(shiftedWaveform, i - start));

   }
   auto trapMax = getMax(filtered, start);
   
   rescaleWaveform(waveform, start, trapMax/2.0);
   
   auto waveHist = createHistogram(*waveform, "unfiltered");
   auto filterHist = createHistogram(filtered, "filtered");


   TCanvas *c = new TCanvas("c1","");
   filterHist->Draw("hist");
   waveHist->Draw("hist SAME");
   filterHist->GetYaxis()->SetRangeUser(-trapMax*0.2, trapMax * 1.1);
}
int removeBaselineAndGetStart(vecFloat *data, int threshold)
{
   float baseline = 0;
   for (int i = 1; i < 21; ++i) {
      baseline += data->at(i) / 20;
   }
   
   int start = -1;
   for (int i = 0; i < data->size(); ++i) {
      data->at(i) -= baseline;
      if (start == -1 && data->at(i) > threshold)
         start = i;
   }
   return start;
}
vecFloat *readFile(std::ifstream &file)
{
   vecFloat *wave = new vecFloat();
   std::string line;

   for (int tb = 0; tb < 512; tb++) {
      if (file.eof())
         break;
      std::getline(file, line);
      std::istringstream str(line);
      float val;
      str >> val;
      wave->push_back(val);
   }
   return wave;
}

TH1F *createHistogram(const vecFloat &data, TString name)
{
   TH1F *hist = new TH1F(name, name, data.size(), 0, data.size());

   for (int i = 0; i < data.size(); ++i)
      hist->Fill(i, data[i]);
   return hist;
}

float d(const vecFloat &signal, int n)
{
   float dN = signal.at(n);
   if (n - k >= 0)
      dN -= signal.at(n - k);
   if (n - l >= 0)
      dN -= signal.at(n - l);
   if (n - k - l >= 0)
      dN += signal.at(n - k - l);
   return dN;
}

float p(const vecFloat &signal, int n)
{
   if (n < 0)
      return 0;
   return p(signal, n - 1) + d(signal, n);
}
float s(const vecFloat &signal, int n)
{
   //if (n < 0 || n > 2*k + l)
   if (n < 0)
      return 0;
   return s(signal, n - 1) + r(signal, n);
}

float r(const vecFloat &signal, int n)
{
   return p(signal, n) + M * d(signal, n);
}

void rescaleWaveform(vecFloat *data, int start, float scale)
{
   auto max = getMax(*data, start);
   scale /= max;

   for(auto &elem : *data)
      elem *= scale;
      
}

int getMax(const vecFloat &data, int start)
{
   float max = 0;
   for(int i = start; i < 10+start; ++i)
      if (data.at(i) > max)
	 max = data.at(i);
   return max;
}
