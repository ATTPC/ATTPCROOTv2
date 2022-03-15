#include  "../helper.h"
#include "TMultiGraph.h"
#include "THStack.h"
#include "Math/Vector3D.h"

/*** Forward declares ***/
using XYZVector = ROOT::Math::XYZVector;
XYZVector calcualteVetrex(const std::vector<XYZVector> &lineStart, const std::vector<XYZVector> &lineStep);
void setAngleAndVertex();
double getHitDistanceFromVertex(const AtHit& hit);
double getHitDistanceFromVertexAlongZ(const AtHit &hit);
void createHistograms(double binWidth);
void clearHists();
void fillHist(double zCut, int numPoints);
void resetAll();
double calcZ1(double R);

/*** Private variable ****/
TMultiGraph *dEdxGraph = nullptr;
XYZVector vertex;
Double_t angle;
double Zcn = 84;

THStack *dEdxStack = nullptr;
TH1F *dEdx[] = {nullptr, nullptr};
THStack *dEdxStackZ = nullptr;
TH1F *dEdxZ[] = {nullptr, nullptr};
TH1F *hdEratio = new TH1F("ratio", "Proxy for Z", 50, -1, 1);
TH1F *hdEratioCorrected = new TH1F("ratioVcorrected", "Proxy for Z velocity corrected", 50, -1, 1);
TH1F *hAngle = new TH1F("angle", "Angle between tracks", 45, 0, 45);
TH1F *hZDistro = new TH1F("hZDistro", "Z", 80, 22, 62);
TH1F *hZDistroCorrected = new TH1F("hZDistroCorrected", "Z velcocity corrected", 80, 22, 62);
TH1F *hZAvg = new TH1F("hZAvg", "Z", 80, 22, 62);
TH1F *hZAvgCorrected = new TH1F("hZAvgCorrected", "Z Corrected", 80, 22, 62);
TH1F *hVInitial = new TH1F("hVInitial", "diff(v)/sum(v)", 100, 0, 1);
TH2F *hAngleVCorr = new TH2F("hAngleVCorr", "Angle vs normalized velocity difference", 45, 0, 45, 200, -1, 1);
TH2F *hAngleVCorrSym = new TH2F("hAngleVCorrSym", "Angle vs normalized velocity difference", 45, 0, 45, 200, -1, 1);
TH2F *hdEratioVCorr = new TH2F("hdEratioVCorr", "dE ratio vs normalized velocity difference", 200, -1, 1, 200, -1, 1);
TH2F *hdEratioVCorrSym =
   new TH2F("hdEratioVCorrSym", "dE ratio vs normalized velocity difference", 200, -1, 1, 200, -1, 1);
TH1F *hZDiff = new TH1F("hZDiff", "Z diff", 10,-5,5);

void determineZ(
   TString fileName =
      "/mnt/simulations/attpcroot/adam/ATTPCROOTv2/macro/e12014/simulation/eventGenerator/sym/output_digi.root",
   double binWidth = 100, TString rawEventName = "AtRawEvent", TString eventName = "AtEventH")
{
   loadRun(fileName, rawEventName, eventName);
   createHistograms(binWidth);
}

void constructdEdx(int eventNum, double angleCut=25, double zCut=600, int numPoints=2)
{
   loadEvent(eventNum);

   // Check the kinematics
   auto tracks = ransacPtr->GetTrackCand(); // type vector<AtTrack>
   if (tracks.size() != 2)
   {
      std::cout << "Event " << eventNum << " does not have 2 tracks. Can't construct dE/dx" << std::endl;
      return;
   }
   setAngleAndVertex();

   if (angle < angleCut || vertex.z() < zCut) {
      std::cout << "Event " << eventNum << " does not pass the cut" << std::endl;
      return;
   }

   clearHists();
   fillHist(0, numPoints);
   
   dEdxStack->Draw("nostack,X0,ep1");
}

void resetAll()
{
   clearHists();
   hdEratio->Reset();
   hdEratioCorrected->Reset();
   hAngle->Reset();
   hZDistro->Reset();
   hZDistroCorrected->Reset();
   hZAvg->Reset();
   hZAvgCorrected->Reset();
   hZDiff->Reset();
   hVInitial->Reset();
   hAngleVCorr->Reset();
   hAngleVCorrSym->Reset();
   hdEratioVCorr->Reset();
   hdEratioVCorrSym->Reset();
}

void fillHist(double zCut, int numPoints)
{

   int Z1 = 0;
   std::vector<double> vel;
   for(int i = 0; i < 2; ++i)
   {
      //Fill the hist with all the points
      //Get the first hit and fill v distro
      auto initialHit = ransacPtr->GetTrackCand()[i].GetHitArray()->at(0);
      auto E = initialHit.GetMCSimPointArray()[0].energy;
      auto A = initialHit.GetMCSimPointArray()[0].A;
      auto vSquared = 2 * E / (A * 931.5);
      Z1 = initialHit.GetMCSimPointArray()[0].Z;
      vel.push_back(TMath::Sqrt(vSquared));

      for(auto &hit : *(ransacPtr->GetTrackCand()[i].GetHitArray()))
	 if(hit.GetPosition().z() > zCut)
	 {
       dEdx[i]->Fill(getHitDistanceFromVertex(hit), hit.GetCharge());
       dEdxZ[i]->Fill(getHitDistanceFromVertexAlongZ(hit), hit.GetCharge());
    }

      for (int bin = 0; bin < dEdx[i]->GetNbinsX(); ++bin) {
         dEdx[i]->SetBinError(bin, TMath::Sqrt(dEdx[i]->GetBinContent(bin)));
         dEdxZ[i]->SetBinError(bin, TMath::Sqrt(dEdxZ[i]->GetBinContent(bin)));
      }
   }

   auto vRatio = (vel[0] - vel[1]) / (vel[0] + vel[1]);

   // Fill angle, velcocity correlation plots
   hVInitial->Fill(vRatio);
   hAngle->Fill(angle);
   hAngleVCorr->Fill(angle, vRatio);
   hAngleVCorrSym->Fill(angle, vRatio);
   hAngleVCorrSym->Fill(angle, -vRatio);

   bool lastBinHadData = false;
   double lastE0 = 0, lastE1 = 0;
   std::vector<double> zVec;
   int numRecorded = 0;
   double sumZ = 0;
   double sumZCorrected = 0;

   // Fill dEdX correlation plots
   for(int bin = 0; bin < dEdx[0]->GetNbinsX(); ++bin)
   {
      auto E0 = dEdx[0]->GetBinContent(bin);
      auto E1 = dEdx[1]->GetBinContent(bin);
      
      bool thisBinHasData = E0 > 2000 && E1 > 2000;
      if(lastBinHadData && thisBinHasData && numRecorded < numPoints)
      {
         // Stop looking if energy jumps by 50% between two points
         if (TMath::Abs(lastE0 - E0) / (E0 + lastE0) > .5 || TMath::Abs(lastE1 - E1) / (E1 + lastE1) > .5)
            break;

         auto R = TMath::Abs((lastE0 - lastE1) / (lastE0 + lastE1));

         auto E0Corr = lastE0 * vel[0] * vel[0];
         auto E1Corr = lastE1 * vel[1] * vel[1];
         auto Rcorr = TMath::Abs((E0Corr - E1Corr) / (E0Corr + E1Corr));

         hdEratio->Fill(R);
         hdEratio->Fill(-R);
         hdEratioCorrected->Fill(Rcorr);
         hdEratioCorrected->Fill(-Rcorr);

         hdEratioVCorr->Fill(R, vRatio);

         hdEratioVCorrSym->Fill(R, vRatio);
         hdEratioVCorrSym->Fill(-R, -vRatio);

         auto z1 = calcZ1(R);
         hZDistro->Fill(z1);
         hZDistro->Fill(Zcn - z1);

         auto z1Corr = calcZ1(Rcorr);
         hZDistroCorrected->Fill(z1Corr);
         hZDistroCorrected->Fill(Zcn - z1Corr);

         hZDiff->Fill(z1 - Z1);
         hZDiff->Fill(Z1 - z1);

         numRecorded++;
         if (z1 < Zcn / 2.)
            sumZ += z1;
         else
            sumZ += Zcn - z1;

         if (z1Corr < Zcn / 2.)
            sumZCorrected += z1Corr;
         else
            sumZCorrected += Zcn - z1Corr;
      }
      
      lastBinHadData = thisBinHasData;
      lastE0 = E0;
      lastE1 = E1;
   }
   std:: cout << sumZ << " " << sumZ/numRecorded << std::endl;
   hZAvg->Fill(sumZ/numRecorded);
   hZAvg->Fill(Zcn - sumZ/numRecorded);
   hZAvgCorrected->Fill(sumZCorrected / numRecorded);
   hZAvgCorrected->Fill(Zcn - sumZCorrected / numRecorded);
}
double calcZ1(double R)
{

   auto comp = 1 - TMath::Sq(R);
   auto sqrtComp = TMath::Sqrt(comp);
   auto pre = Zcn/(2.*R);

   // For p same and A1 = Z1/Z*A	 
   //return pre*(R-1-sqrtComp*(1 - TMath::Sqrt(2*(1+1/sqrtComp))));

   //For v is same
   return pre*(R-1 + sqrtComp);

}
void clearHists()
{
   for (int i = 0; i < 2; ++i) {
      dEdx[i]->Reset();
      dEdxZ[i]->Reset();
   }
}
void createHistograms(double binWidth)
{
   double maxLength = TMath::Sqrt(TMath::Sq(1000) + TMath::Sq(25));
   int nBins = TMath::CeilNint(maxLength/binWidth);

   std::cout << "Creating dE/dx curve over " << nBins << " bins" << std::endl;
   for (int i = 0; i < 2; ++i) {
      if(dEdx[i] != nullptr)
	 delete dEdx[i];
      if (dEdxZ[i] != nullptr)
         delete dEdxZ[i];
   }

   if(dEdxStack != nullptr)
      delete dEdxStack;
   if (dEdxStackZ != nullptr)
      delete dEdxStackZ;

   dEdxStack = new THStack("hs", "Stacked dE/dx curves");
   dEdxStackZ = new THStack("hsz", "Stacked dE/dx curves bin in Z");

   dEdx[0] = new TH1F("dEdx1", "dEdX Frag 1", nBins, 0, maxLength);
   dEdx[0]->SetMarkerStyle(21);
   dEdx[0]->SetMarkerColor(9);
   dEdx[1] = new TH1F("dEdx2", "dEdX Frag 2", nBins, 0, maxLength);
   dEdx[1]->SetMarkerStyle(21);
   dEdx[1]->SetMarkerColor(31);

   dEdxStack->Add(dEdx[0]);
   dEdxStack->Add(dEdx[1]);

   dEdxZ[0] = new TH1F("dEdxZ1", "dEdX Frag 1", nBins, 0, maxLength);
   dEdxZ[0]->SetMarkerStyle(21);
   dEdxZ[0]->SetMarkerColor(9);
   dEdxZ[1] = new TH1F("dEdxZ2", "dEdX Frag 2", nBins, 0, maxLength);
   dEdxZ[1]->SetMarkerStyle(21);
   dEdxZ[1]->SetMarkerColor(31);

   dEdxStackZ->Add(dEdxZ[0]);
   dEdxStackZ->Add(dEdxZ[1]);
}

double getHitDistanceFromVertex(const AtHit& hit)
{
   auto p = hit.GetPosition();
   auto position = XYZVector(p.x(), p.y(), p.z());
   auto diff = position - vertex;
   return TMath::Sqrt(diff.Mag2());
}
double getHitDistanceFromVertexAlongZ(const AtHit &hit)
{
   auto p = hit.GetPosition();
   auto diff = p.z() - vertex.z();
   return TMath::Abs(diff);
}
void setAngleAndVertex()
{
   std::vector<XYZVector> lineStart;
   std::vector<XYZVector> lineStep; // each step is 1 mm in z

   for( auto &track : ransacPtr->GetTrackCand() )
   {
      lineStart.emplace_back(XYZVector(track.GetFitPar()[0], track.GetFitPar()[2], 0));
      lineStep.emplace_back(XYZVector(track.GetFitPar()[1], track.GetFitPar()[3], 1));
   }
   vertex = calcualteVetrex(lineStart, lineStep);

   auto dot = lineStep[0].Unit().Dot(lineStep[1].Unit());
   angle = TMath::ACos(dot) * TMath::RadToDeg();
}

XYZVector calcualteVetrex(const std::vector<XYZVector> &lineStart, const std::vector<XYZVector> &lineStep)
{
   auto n = lineStep[0].Cross(lineStep[1]);
   auto n0 = lineStep[0].Cross(n);
   auto n1 = lineStep[1].Cross(n);

   auto c0 = lineStart[0] + (lineStart[1] - lineStart[0]).Dot(n1)/lineStep[0].Dot(n1) * lineStep[0];
   auto c1 = lineStart[1] + (lineStart[0] - lineStart[1]).Dot(n0)/lineStep[1].Dot(n0) * lineStep[1];

   return (c0 + c1)/2.;
}
