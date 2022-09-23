
#include <TCanvas.h>
#include <TComplex.h>
#include <TF1.h>
#include <TFile.h>
#include <TFitResult.h>
#include <TFitResultPtr.h>
#include <TGraph.h>
#include <TGraphErrors.h>
#include <TH2.h>
#include <TRandom3.h>
#include <TVirtualFFT.h>

#include <fstream>
#include <numeric>
#ifndef __CLING__
#include <FairParAsciiFileIo.h>
#include <FairRunAna.h>
#include <FairRuntimeDb.h>

#include "../build/include/AtContainerManip.h"
#include "../build/include/AtDigiPar.h"
#include "../build/include/AtHit.h"
#include "../build/include/AtPSADeconv.h"
#include "../build/include/AtPadArray.h"
#include "../build/include/AtPadFFT.h"
#include "../build/include/AtRawEvent.h"

#endif

auto run = std::make_unique<FairRunAna>();
constexpr int nX = 4;
TCanvas *cFFT = nullptr;
TCanvas *cDifference = nullptr;
TCanvas *cQRatio = nullptr;

TGraph *gr = nullptr;
TGraph *grMax = nullptr;

std::vector<TH1F *> hTime;
std::vector<TH1F *> hFreq;
auto hDiff = new TH1F(TString::Format("diff"), TString::Format("Charge Difference"), 512, 0, 511);

std::vector<Int_t> dimSize = {512};
auto fFFT = std::unique_ptr<TVirtualFFT>(TVirtualFFT::FFT(1, dimSize.data(), "R2C M K"));

void deconvolution()
{
   TString parFile = gSystem->Getenv("VMCWORKDIR") + TString("/parameters/ATTPC.e12014.par");

   FairParAsciiFileIo *parIo1 = new FairParAsciiFileIo();
   parIo1->open(parFile, "in");
   auto fPar = dynamic_cast<AtDigiPar *>(run->GetRuntimeDb()->getContainer("AtDigiPar"));
   fPar->init(parIo1);
}
// Get input from meaasured signal and response function
void canvas()
{
   if (cFFT == nullptr) {
      cFFT = new TCanvas("cFFT", "Deconvolution", 300 * nX, 600);
      cFFT->Divide(nX, 2);
   }
   if (cDifference == nullptr) {
      cDifference = new TCanvas("cDif", "Q Difference", 600, 400);
   }
   if (cQRatio == nullptr) {
      cQRatio = new TCanvas("cRatio", "Q Ratio", 600, 400);
   }

   while (hTime.size() < nX) {
      int id = hTime.size();
      hTime.push_back(new TH1F(TString::Format("time_%d", id), TString::Format("Time %d", id), 512, 0, 511));
   }
   while (hFreq.size() < nX) {
      int id = hFreq.size();
      hFreq.push_back(
         new TH1F(TString::Format("freq_%d", id), TString::Format("Frequency %d", id), 512 / 2 + 1, 0, 512 / 2));
   }

   for (int i = 0; i < nX * 2; ++i) {
      cFFT->cd(i + 1);
      int x = i % nX;
      int y = i / nX;
      if (y == 0)
         hTime[x]->Draw("hist");
      if (y == 1)
         hFreq[x]->Draw("hist");
   }
   cDifference->cd();
   hDiff->Draw("hist");
}

AtPad GetTriangleResponse(int base)
{
   AtPad::trace wave;
   wave.fill(0);
   double dx = 1.0 / base;
   int iniBase = base;
   while (base > 0) {
      wave[iniBase - base] = dx * base;
      --base;
   }

   AtPad response;
   response.SetADC(wave);
   fFFT->SetPoints(response.GetADC().data());
   fFFT->Transform();
   response.AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));

   return response;
}

AtPad GetNominalResponse(double par = 3)
{
   AtPad::trace wave;

   TF1 responseFunc("fR", "exp(-[0]*x/[1])*sin(x/[1])*(x/[1])**3", 0, 512);

   responseFunc.SetParameter(0, par);
   responseFunc.SetParameter(1, 0.72 / 0.32);
   double scale = responseFunc.GetMaximum();

   for (int i = 0; i < wave.size(); ++i)
      wave[i] = responseFunc(i + 0.5) / scale;

   AtPad response;
   response.SetADC(wave);
   fFFT->SetPoints(response.GetADC().data());
   fFFT->Transform();
   response.AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));

   return response;
}

AtPad GetFileResponse(
   std::string fileName = "/mnt/analysis/e12014/huntc/newFork/ATTPCROOTv2/code/macros/waveSample.txt",
   double scale = 10)
{
   AtPad::trace wave;
   wave.fill(0);
   std::ifstream file(fileName);
   if (!file.is_open())
      return {};

   int i = 0;
   while (!file.eof()) {
      double val;
      file >> val;
      wave[i++] = val * scale;
   }
   AtPad response;
   response.SetADC(wave);
   fFFT->SetPoints(response.GetADC().data());
   fFFT->Transform();
   response.AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));

   return response;
}

AtPad GetInputStep(int min, int max)
{
   AtPad::trace wave;
   wave.fill(0);
   for (int i = min; i < max; ++i)
      wave[i] = 1;

   AtPad input;
   input.SetADC(wave);
   fFFT->SetPoints(input.GetADC().data());
   fFFT->Transform();
   input.AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));

   return input;
}

AtPad GetInputGaus(double mean, double sigma)
{
   AtPad::trace wave;
   wave.fill(0);
   TF1 f1("fInput", "gaus(0)", 0, 512);
   f1.SetParameter(0, 1);
   f1.SetParameter(1, mean);
   f1.SetParameter(2, sigma);
   for (int i = 0; i < 512; ++i)
      wave[i] = f1.Eval(i);

   AtPad input;
   input.SetADC(wave);
   fFFT->SetPoints(input.GetADC().data());
   fFFT->Transform();
   input.AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));

   return input;
}

AtPad GetSignal(const AtPad &input, const AtPad &response, double sigma = 0)
{
   AtPad::trace wave;
   wave.fill(0);
   for (int i = 0; i < 512; ++i)
      for (int j = i; j < 512; ++j)
         wave[j] += input.GetADC(i) * response.GetADC(j - i);
   if (sigma != 0)
      for (int i = 0; i < 512; ++i)
         wave[i] += gRandom->Gaus(0, sigma);

   AtPad signal;
   signal.SetADC(wave);
   fFFT->SetPoints(signal.GetADC().data());
   fFFT->Transform();
   signal.AddAugment("fft", AtPadFFT::CreateFromFFT(fFFT.get()));
   return signal;
}

// Only works for even-valued orders
double LowPassFilter(int order, double cuttoff, TComplex freq)
{
   if (order == 0)
      return 1;

   order /= 2;

   cuttoff *= cuttoff;
   auto freqMag = freq * TComplex::Conjugate(freq);
   double reducedFreq = freqMag.Re() / cuttoff;
   return 1.0 / (1.0 + std::pow(reducedFreq, order));
}

// Actually do the deconvolution with a signal and response
std::unique_ptr<AtPad> Divide(AtPad &signal, const AtPad &response, int order = 4, double freq = 25)
{
   // Create the FFT object we will have to use to transform back into the time domain.
   std::vector<int> dimSize = {512};
   auto virtFFT = std::unique_ptr<TVirtualFFT>(TVirtualFFT::FFT(1, dimSize.data(), "C2R M K"));

   // Set the FFT with the data before transforming back

   // Create the charge information and fft information
   auto q = dynamic_cast<AtPadArray *>(signal.AddAugment("Qreco", std::make_unique<AtPadArray>()));
   auto qfft = dynamic_cast<AtPadFFT *>(signal.AddAugment("Qreco-fft", std::make_unique<AtPadFFT>()));

   auto signalFFT = dynamic_cast<const AtPadFFT *>(signal.GetAugment("fft"));
   auto responseFFT = dynamic_cast<const AtPadFFT *>(response.GetAugment("fft"));

   bool thresh = false;
   for (int i = 0; i < 512 / 2 + 1; ++i) {
      auto a = signalFFT->GetPoint(i);
      auto b = responseFFT->GetPoint(i);

      TComplex bComp = TComplex(b.first, b.second);
      auto z = TComplex(a.first, a.second) / bComp;
      auto filt = LowPassFilter(order, freq, TComplex(0, i));

      // std::cout << i << " " << bComp << " " << filt << " " << filt / bComp << std::endl;
      z *= LowPassFilter(order, freq, TComplex(0, i));
      qfft->SetPointIm(i, z.Im());
      qfft->SetPointRe(i, z.Re());

      //<< " " << TComplex(signal.GetPoint(i).first, signal.GetPoint(i).second)
      // std::cout << i << " " << TComplex(a.first, a.second) << " " << filt / bComp << " " << z << std::endl;

      z /= 512;
      virtFFT->SetPointComplex(i, z);
   }

   virtFFT->Transform();

   for (int i = 0; i < signal.GetADC().size(); ++i)
      q->SetArray(i, virtFFT->GetPointReal(i));

   double baseline = std::accumulate(q->GetArray().begin(), q->GetArray().begin() + 20, 0.0);
   baseline /= 20;
   for (int i = 0; i < q->GetArray().size(); ++i)
      q->SetArray(i, virtFFT->GetPointReal(i) - baseline);

   return nullptr;
}

void deconvolutionSingle(double noise = 0, int order = 4, double cutoff = 25, double width = 10)
{
   // Create the response pad
   AtPad response = GetNominalResponse();
   // AtPadFFT response = GetFileResponse();
   //   AtPadFFT input = GetInputStep(10, 50);
   AtPad input = GetInputGaus(250, width);
   // AtPadFFT input = GetInputGaus(100, 50);
   AtPad signal = GetSignal(input, response, noise * 3 / 2.355);

   auto recoPad = Divide(signal, response, order, cutoff);
   canvas();

   // Fill the histograms
   hDiff->SetBins(512, 0, 512);
   auto charge = dynamic_cast<AtPadArray *>(signal.GetAugment("Qreco"))->GetArray();

   for (int i = 0; i < 512; ++i) {
      hTime[0]->SetBinContent(i + 1, input.GetADC(i));
      hTime[1]->SetBinContent(i + 1, response.GetADC(i));
      hTime[2]->SetBinContent(i + 1, signal.GetADC(i));
      hTime[3]->SetBinContent(i + 1, charge[i]);
      if (i < 512 / 2 + 1) {
         hFreq[0]->SetBinContent(i + 1, dynamic_cast<AtPadFFT *>(input.GetAugment("fft"))->GetPointMag(i));
         hFreq[1]->SetBinContent(i + 1, dynamic_cast<AtPadFFT *>(response.GetAugment("fft"))->GetPointMag(i));
         hFreq[2]->SetBinContent(i + 1, dynamic_cast<AtPadFFT *>(signal.GetAugment("fft"))->GetPointMag(i));
         hFreq[3]->SetBinContent(i + 1, dynamic_cast<AtPadFFT *>(signal.GetAugment("Qreco-fft"))->GetPointMag(i));
      }
      hDiff->SetBinContent(i + 1, charge[i] - input.GetADC(i));
   }

   canvas();
}

void deconvolutionSingleNew(double noise = 0, int order = 4, double cutoff = 25, double width = 10, double par = 3)
{
   // Create the response pad
   // AtPadFFT response = GetFileResponse();

   auto fPar = dynamic_cast<AtDigiPar *>(run->GetRuntimeDb()->getContainer("AtDigiPar"));
   auto psa = std::make_unique<AtPSADeconv>();
   psa->SetResponse(ElectronicResponse::AtNominalResponse(fPar->GetPeakingTime() / 1000.));
   psa->SetFilterOrder(order);
   psa->SetCutoffFreq(cutoff);
   psa->Init();

   std::cout << "Init finished" << std::endl;

   const AtPad &response = psa->GetResponse(1);
   AtPad input = GetInputGaus(250, width);
   AtPad signal = GetSignal(input, response, noise * 3 / 2.355);
   signal.SetPadNum(1);

   std::cout << "Created input/output pads " << std::endl;
   auto hits = psa->AnalyzePad(&signal);
   for (auto &hit : hits) {
      std::cout << "Hit: " << hit->GetHitID();
      std::cout << "Q: " << hit->GetCharge() << " Pos: " << hit->GetPosition() << " +- " << hit->GetPositionSigma()
                << std::endl;
   }

   canvas();
   // Fill the histograms
   hDiff->SetBins(512 / 2 + 1, 0, 512 / 2 + 1);

   auto charge = dynamic_cast<AtPadArray *>(signal.GetAugment("Qreco"))->GetArray();
   for (int i = 0; i < 512; ++i) {
      hTime[0]->SetBinContent(i + 1, input.GetADC(i));
      hTime[1]->SetBinContent(i + 1, response.GetADC(i));
      hTime[2]->SetBinContent(i + 1, signal.GetADC(i));
      hTime[3]->SetBinContent(i + 1, charge[i]);
      if (i < 512 / 2 + 1) {
         hFreq[0]->SetBinContent(i + 1, dynamic_cast<AtPadFFT *>(input.GetAugment("fft"))->GetPointMag(i));
         hFreq[1]->SetBinContent(i + 1, dynamic_cast<const AtPadFFT *>(response.GetAugment("fft"))->GetPointMag(i));
         hFreq[2]->SetBinContent(i + 1, dynamic_cast<AtPadFFT *>(signal.GetAugment("fft"))->GetPointMag(i));
         hFreq[3]->SetBinContent(i + 1, dynamic_cast<AtPadFFT *>(signal.GetAugment("Qreco-fft"))->GetPointMag(i));
      }
      hDiff->SetBinContent(i + 1, charge[i] - input.GetADC(i));
   }

   canvas();
}

double GetdQ(const AtPad &input, const AtPad &reco)
{
   double dQ = 0;
   auto charge = dynamic_cast<const AtPadArray *>(reco.GetAugment("Qreco"));
   for (int i = 0; i < 512; ++i)
      dQ += std::pow(input.GetADC(i) - charge->GetArray(i), 2);
   dQ *= 512;
   dQ = std::sqrt(dQ);
   return dQ;
}

void deconvolutionFixedWidth(double noiseLevel = 0, int order = 6, double width = 10)
{
   // Setup the problem
   AtPad response = GetFileResponse();
   AtPad input = GetInputGaus(250, width);
   double Q = std::accumulate(input.GetADC().begin(), input.GetADC().end(), 0.0);

   double sigma = noiseLevel * (*std::max_element(input.GetADC().begin(), input.GetADC().end())) / 2.355;
   std::cout << Q << " " << noiseLevel << " " << sigma << std::endl;

   // Cuttoff frequencies to plot
   std::vector<double> cutoff = {5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 75, 100};
   std::vector<double> ratio;
   std::vector<double> max;
   std::vector<double> errorY;
   std::vector<double> errorYMax;
   std::vector<double> errorX;

   // Get the summary statistics for each cutoff frequencies
   for (auto freq : cutoff) {

      // Generate 100 events and dQ
      std::cout << std::endl << "Fiiting cutoff " << freq << std::endl;
      std::vector<double> dQ;
      std::vector<double> maxQ;

      for (int i = 0; i < 100; ++i) {
         AtPad signal = GetSignal(input, response, sigma);

         Divide(signal, response, order, freq);

         auto &charge = dynamic_cast<const AtPadArray *>(signal.GetAugment("Qreco"))->GetArray();

         auto gaus = std::make_unique<TF1>("fitFunc", "gaus(0)");

         auto itMax = std::max_element(charge.begin(), charge.end());

         auto itHalfMax = itMax;
         while (*itHalfMax > *itMax / 2)
            itHalfMax++;
         int idxMax = itMax - charge.begin();
         int idxFWHM = itHalfMax - itMax;

         auto fit = ContainerManip::CreateHistFromData("temp", charge)
                       ->Fit(gaus.get(), "SRQ", "", idxMax - idxFWHM, idxMax + idxFWHM);

         double norm = fit->Parameter(0);
         double sig = std::abs(fit->Parameter(2));
         double qReco = std::sqrt(3.14159 * 2) * sig * norm;
         double adcMax = *std::max_element(signal.GetADC().begin(), signal.GetADC().end());
         maxQ.push_back(adcMax / Q);
         // dQ.push_back(GetdQ(input, reco));
         dQ.push_back(qReco / Q);
      }

      // Get the summary statistics from all events
      auto mean = std::accumulate(dQ.begin(), dQ.end(), 0.0) / dQ.size();
      double var = 0;
      for (auto val : dQ)
         var += (val - mean) * (val - mean) / dQ.size();
      var = std::sqrt(var);
      ratio.push_back(mean);
      errorY.push_back(var);

      mean = std::accumulate(maxQ.begin(), maxQ.end(), 0.0) / maxQ.size();
      var = 0;
      for (auto val : maxQ)
         var += (val - mean) * (val - mean) / maxQ.size();
      var = std::sqrt(var);
      std::cout << mean << "+-" << var << std::endl;
      max.push_back(mean);
      errorYMax.push_back(var);

      errorX.push_back(0);
   }

   // Create and draw the graph
   if (gr != nullptr)
      delete gr;
   gr = new TGraphErrors(cutoff.size(), cutoff.data(), ratio.data(), errorX.data(), errorY.data());
   gr->SetTitle("R_Q (deconv) vs cuttoff freq");

   if (grMax != nullptr)
      delete grMax;
   grMax = new TGraphErrors(cutoff.size(), cutoff.data(), max.data(), errorX.data(), errorYMax.data());
   grMax->SetTitle("R_Q (max) vs cuttoff freq");

   canvas();
   cQRatio->cd();
   gr->Draw("ACP");
   cDifference->cd();
   grMax->Draw("ACP");
   return;
}

void deconvolutionFixedCutoff(double noiseLevel = 0, int order = 6, double cutoff = 40, double par = 3,
                              bool plotWidth = false)
{
   // Setup the problem
   AtPad response = GetNominalResponse();

   std::vector<double> widths = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
   std::vector<double> ratio;
   std::vector<double> max;
   std::vector<double> errorY;
   std::vector<double> errorYMax;
   std::vector<double> errorX;

   for (auto width : widths) {
      AtPad input = GetInputGaus(250, width);
      double Q = std::accumulate(input.GetADC().begin(), input.GetADC().end(), 0.0);
      double sigma = noiseLevel * (*std::max_element(input.GetADC().begin(), input.GetADC().end())) / 2.355;

      // Generate 100 events and dQ
      std::cout << std::endl << "Fitting width " << width << std::endl;
      std::vector<double> dQ;
      std::vector<double> maxQ;

      //      auto convResp = GetNominalResponse(par);
      auto convResp = GetFileResponse();
      for (int i = 0; i < 100; ++i) {
         AtPad signal = GetSignal(input, convResp, sigma);
         Divide(signal, response, order, cutoff);

         auto &charge = dynamic_cast<const AtPadArray *>(signal.GetAugment("Qreco"))->GetArray();

         auto gaus = std::make_unique<TF1>("fitFunc", "gaus(0)");
         auto itMax = std::max_element(charge.begin(), charge.end());

         auto itHalfMax = itMax;
         while (*itHalfMax > *itMax / 2)
            itHalfMax++;
         int idxMax = itMax - charge.begin();
         int idxFWHM = itHalfMax - itMax;
         TH1F hist("hist", "temp", 512, 0, 512);
         ContainerManip::SetHistFromData(hist, charge);

         auto fit = hist.Fit(gaus.get(), "SRQ", "", idxMax - idxFWHM, idxMax + idxFWHM);

         double norm = fit->Parameter(0);
         double sig = std::abs(fit->Parameter(2));
         double qReco = std::sqrt(3.14159 * 2) * sig * norm;

         if (!plotWidth) {
            double adcMax = *std::max_element(signal.GetADC().begin(), signal.GetADC().end());
            maxQ.push_back(adcMax / Q / 0.15);
         } else
            maxQ.push_back(sig);

         // dQ.push_back(GetdQ(input, reco));
         dQ.push_back(qReco / Q);
      }

      // Get the summary statistics from all events
      auto mean = std::accumulate(dQ.begin(), dQ.end(), 0.0) / dQ.size();
      double var = 0;
      for (auto val : dQ)
         var += (val - mean) * (val - mean) / dQ.size();
      var = std::sqrt(var);
      ratio.push_back(mean);
      errorY.push_back(var);

      mean = std::accumulate(maxQ.begin(), maxQ.end(), 0.0) / maxQ.size();
      var = 0;
      for (auto val : maxQ)
         var += (val - mean) * (val - mean) / maxQ.size();
      var = std::sqrt(var);
      std::cout << mean << "+-" << var << std::endl;
      max.push_back(mean);
      errorYMax.push_back(var);

      errorX.push_back(0);
   }

   // Create and draw the graph
   if (gr != nullptr)
      delete gr;
   gr = new TGraphErrors(widths.size(), widths.data(), ratio.data(), errorX.data(), errorY.data());
   gr->SetTitle("R_Q (deconv) vs input width (sigma)");

   if (grMax != nullptr)
      delete grMax;
   grMax = new TGraphErrors(widths.size(), widths.data(), max.data(), errorX.data(), errorYMax.data());
   if (plotWidth)
      grMax->SetTitle("Reconstructed vs input width (sigma)");
   else
      grMax->SetTitle("R_Q (max) vs input width (sigma)");

   canvas();
   cQRatio->cd();
   gr->Draw("ACP");
   cDifference->cd();
   grMax->Draw("ACP");
   return;
}
