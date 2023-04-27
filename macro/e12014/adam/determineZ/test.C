#include "TH1F.h"
#include "TRandom3.h"

#include <iostream>
#include <thread>
#include <vector>
const int nthreads = 4;
const int nevents = 10000;
std::vector<TH1F *> histograms(nthreads);
std::mutex coutMutex;
void fill_histogram(TH1F *h)
{
   TRandom3 rand(0);
   for (int i = 0; i < nevents; i++) {
      h->Fill(rand.Gaus(0, 1));
   }
}

void fit_histogram(TH1F *h)
{

   // TF1 gauss(TString::Format("%lu", id), "gaus(0)", -3, 3, TF1::EAddToList::kNo);
   TF1 gauss("gauss", "gaus(0)", -3, 3, TF1::EAddToList::kNo);
   gauss.SetParameter(0, 100);
   gauss.SetParameter(1, 0);
   gauss.SetParameter(2, 1);

   // Create the data to fit
   double xmin = 0, xmax = 0;
   gauss.GetRange(xmin, xmax);
   ROOT::Fit::DataOptions opt;
   ROOT::Fit::DataRange range(xmin, xmax);
   ROOT::Fit::BinData d(opt, range);
   ROOT::Fit::FillData(d, h);

   // Create the function to fit
   ROOT::Math::WrappedMultiTF1 wf(gauss);

   ROOT::Fit::Fitter fitter;
   fitter.Config().SetMinimizer("Minuit2");
   fitter.SetFunction(wf, false);

   bool ret = fitter.Fit(d);
   const ROOT::Fit::FitResult *resultPtr = nullptr;
   if (ret)
      resultPtr = &fitter.Result();
   else {
      lock_guard<mutex> lock(coutMutex);
      cout << "Failed to fit histogram" << endl;
      return;
   }

   resultPtr = AtPSADeconvFit::FitHistorgramParallel(*h, gauss);

   //   auto resultPtr = h->Fit("gaus", "SQN");
   auto amp = resultPtr->GetParams()[0];
   auto z = resultPtr->GetParams()[1];
   auto sig = resultPtr->GetParams()[2];

   auto Q = amp * sig * std::sqrt(2 * TMath::Pi());
   {
      lock_guard<mutex> lock(coutMutex);
      cout << "Fit: " << amp << " " << z << " " << sig << " " << endl;
   }
}
void fit_histogram_serial(TH1F *h)
{
   auto id = std::hash<std::thread::id>{}(std::this_thread::get_id());
   TF1 gauss(TString::Format("%lu", id), "gaus(0)", -3, 3, TF1::EAddToList::kNo);
   gauss.SetParameter(0, 100);
   gauss.SetParameter(1, 0);
   gauss.SetParameter(2, 1);

   auto resultPtr = h->Fit(&gauss, "SQN");
   auto amp = resultPtr->GetParams()[0];
   auto z = resultPtr->GetParams()[1];
   auto sig = resultPtr->GetParams()[2];

   auto Q = amp * sig * std::sqrt(2 * TMath::Pi());
   {
      lock_guard<mutex> lock(coutMutex);
      cout << "Fit: " << amp << " " << z << " " << sig << " " << endl;
   }
}

int test()
{
   ROOT::EnableThreadSafety();

   for (int i = 0; i < nthreads; i++) {
      histograms[i] = new TH1F(Form("h%d", i), "", 100, -5, 5);
      fill_histogram(histograms[i]);
   }
   std::vector<std::thread> threads(nthreads);
   for (int i = 0; i < nthreads; i++) {
      threads[i] = std::thread(fit_histogram, histograms[i]);
      // fit_histogram(histograms[i]);
   }
   for (int i = 0; i < nthreads; i++) {
      threads[i].join();
   }

   std::cout << endl;
   for (int i = 0; i < nthreads; i++) {
      // threads[i] = std::thread(fit_histogram, histograms[i]);
      fit_histogram_serial(histograms[i]);
   }

   return 0;
}
