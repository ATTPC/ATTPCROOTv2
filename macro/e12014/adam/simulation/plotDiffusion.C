#include "../helper.h"
#include "../build/include/AtDecoder/AtTrack.h"
#include "TH2.h"
#include "TF2.h"
#include "TGraph2DErrors.h"
#include "TGraphErrors.h"

struct cluster {
   Double_t x, xSig;
   Double_t y, ySig;
   Double_t z, zSig;
};
AtTrack *trackPtr = nullptr;

void setTrack(int trackID);
void drawTrack();
void drawSubtrack(float zMin, float zMax);
void drawAndFitSubtrack(float zMin, float zMax);
double weightedMean(const std::vector<double> &x, const std::vector<double> &w);
double weightedStdDev(const std::vector<double> &x, const std::vector<double> &w, double avg = 0.0);
bool compareTrack(const AtTrack &a, const AtTrack &b);

void plotDiffusion(TString run, ULong64_t eventNumber, Int_t trackID = 0)
{
   loadRun(run, "AtRawEvent", "AtEventH");
   loadEvent(eventNumber);
   setTrack(trackID);
   std::cout << "Using track with " << trackPtr->GetHitArray()->size() << " hits" << std::endl;
   drawTrack();
}

void setTrack(int trackID)
{
   auto trackList = ransacPtr->GetTrackCand();
   std::sort(trackList.begin(), trackList.end(), compareTrack);
   trackPtr = new AtTrack(trackList.at(trackID));
}

void drawTrack()
{
   auto padPlane = tpcMap->GetAtTpcPlane();
   padPlane->ClearBinContents();
   for (auto &hit : *trackPtr->GetHitArray())
      padPlane->Fill(hit.GetPosition().x(), hit.GetPosition().y(), hit.GetCharge());
   padPlane->Draw("COL L0");
   padPlane->SetMinimum(1.0);
}

void drawSubtrack(float zMin, float zMax)
{
   auto padPlane = tpcMap->GetAtTpcPlane();
   padPlane->ClearBinContents();
   for (auto &hit : *trackPtr->GetHitArray())
      if (hit.GetPosition().z() > zMin && hit.GetPosition().z() < zMax)
         padPlane->Fill(hit.GetPosition().x(), hit.GetPosition().y(), hit.GetCharge());
   padPlane->Draw("COL L0");
   padPlane->SetMinimum(1.0);
}

void drawAndFitSubtrack(float zMin, float zMax)
{
   std::vector<std::pair<double, double>> position;
   std::vector<double> charge;
   std::vector<double> z;

   for (auto &hit : *trackPtr->GetHitArray())
      if (hit.GetPosition().z() > zMin && hit.GetPosition().z() < zMax) {
         position.push_back({hit.GetPosition().x(), hit.GetPosition().y()});
         charge.push_back(hit.GetCharge());
         z.push_back(hit.GetPosition().z());
      }
   if (z.size() == 0) {
      std::cout << "There are no hits between " << zMin << " and " << zMax << std::endl;
      return;
   }

   TGraph2DErrors *graph = new TGraph2DErrors(position.size());

   int maxIndex = 0;
   for (int i = 0; i < position.size(); ++i) {
      graph->SetPoint(i, position[i].first, position[i].second, charge[i]);
      graph->SetPointError(i, 0.5 / 4., 0.5 / 2., TMath::Sqrt(charge[i]));
      if (charge[maxIndex] < charge[i])
         maxIndex = i;
   }
   TF2 *f2 = new TF2("f2", "[0]*TMath::Gaus(x,[1],[2])*TMath::Gaus(y,[3],[4])", -250, 250, -250, 250);
   f2->SetParameters(*std::max(charge.begin(), charge.end()), position[maxIndex].first, 2, position[maxIndex].second,
                     2);
   graph->Fit(f2);
   graph->Draw("err p0");
   f2->SetNpx(100);
   f2->SetNpy(100);
   f2->SetMinimum(15.);
   f2->Draw("colz");

   double zAvg = weightedMean(z, charge);
   double zSig = weightedStdDev(z, charge, zAvg);
   std::cout << "X: " << f2->GetParameter(1) << " +/- " << f2->GetParameter(2) << std::endl;
   std::cout << "Y: " << f2->GetParameter(3) << " +/- " << f2->GetParameter(4) << std::endl;
   std::cout << "Z: " << zAvg << " +/- " << zSig << std::endl;
}

bool compareTrack(const AtTrack &a, const AtTrack &b)
{
   return a.GetHitArrayConst().size() > b.GetHitArrayConst().size();
}
double weightedMean(const std::vector<double> &x, const std::vector<double> &w)
{
   double totalWeight = 0;
   for (int i = 0; i < w.size(); ++i)
      totalWeight += w[i];

   double mean;
   for (int i = 0; i < w.size(); ++i)
      mean += x[i] * w[i];
   mean /= totalWeight;
   return mean;
}
double weightedStdDev(const std::vector<double> &x, const std::vector<double> &w, double mean)
{
   if (mean == 0)
      mean = weightedMean(x, w);
   double denom = 0;
   int numWeights = 0;
   for (const auto &elem : w) {
      denom += elem;
      if (elem != 0)
         numWeights++;
   }
   denom *= (double)(numWeights - 1) / numWeights;

   double numer = 0;
   for (int i = 0; i < x.size(); ++i)
      numer += w[i] * (x[i] - mean) * (x[i] - mean);
   return TMath::Sqrt(numer / denom);
}
