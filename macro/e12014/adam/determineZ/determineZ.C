#include  "../helper.h"
#include "TMultiGraph.h"
#include "THStack.h"
#include "Math/Vector3D.h"

/*** Forward declares ***/
using XYZVector = ROOT::Math::XYZVector;
XYZVector calcualteVetrex();
double setLineVectorsAndVertex();
double getHitDistanceFromVertex(const AtHit& hit);
void createHistograms(double binWidth);
void clearHists();
void fillHist(double zCut, int numPoints);
void resetAll();
double calcZ1(double R);

/*** Private variable ****/
TMultiGraph *dEdxGraph = nullptr;
std::vector<XYZVector> lineStart;
std::vector<XYZVector> lineStep; // each step is 1 mm in z
XYZVector vertex;
double Zcn = 84;

THStack *dEdxStack = nullptr;
TH1F *dEdx[] = {nullptr, nullptr};
TH1F *hdEratio = new TH1F("ratio", "% diff at same x", 50, -1, 1);
TH1F *hAngle = new TH1F("angle", "Angle between tracks", 45, 0, 45);
TH1F *hZDistro = new TH1F("hZDistro", "Z", 80, 22, 62);
TH1F *hZAvg = new TH1F("hZAvg", "Z", 80, 22, 62);
TH1F *hVInitial = new TH1F("hVInitial", "v",100,0,1);
TH1F *hZDiff = new TH1F("hZDiff", "Z diff", 10,-5,5);

void determineZ(TString fileName = "/mnt/simulations/attpcroot/adam/ATTPCROOTv2/macro/e12014/simulation/eventGenerator/sym/output_digi.root", double binWidth = 100, TString rawEventName = "AtRawEvent", TString eventName = "AtEventH")
{
   loadRun(fileName, rawEventName, eventName);
   createHistograms(binWidth);
}

void constructdEdx(int eventNum, double angleCut, double zCut, int numPoints)
{
   loadEvent(eventNum);

   // Check the kinematics
   auto tracks = ransacPtr->GetTrackCand(); // type vector<AtTrack>
   if (tracks.size() != 2)
   {
      std::cout << "Event does not have 2 tracks. Can't construct dE/dx" << std::endl;
      return;
   }
   
   auto angle = setLineVectorsAndVertex();
   if(angle < angleCut || vertex.z() < zCut)
      return;
   hAngle->Fill(angle);
   // std::cout << "Vertex at: " << vertex << " mm" << std::endl;

   clearHists();
   fillHist(0, numPoints);
   
   dEdxStack->Draw("nostack,X0,ep1");
}

void resetAll()
{
   clearHists();
   hdEratio->Reset();
   hAngle->Reset();
   hZDistro->Reset();
   hZAvg->Reset();
   hZDiff->Reset();
   hVInitial->Reset();
}

void fillHist(double zCut, int numPoints)
{

   int Z1 = 0;
   for(int i = 0; i < 2; ++i)
   {
      //Fill the hist with all the points
      //Get the first hit and fill v distro
      //auto hit = ransacPtr->GetTrackCand()[i].GetHitArray()->at(0);
      for(auto &hit : *(ransacPtr->GetTrackCand()[i].GetHitArray()))
      {
	 if(hit.GetPosition().z() > zCut)
	    dEdx[i]->Fill(getHitDistanceFromVertex(hit), hit.GetCharge());
	 for(int i = 0; i < hit.GetMCSimPointArray().size(); ++i)
	 {
	    auto E = hit.GetMCSimPointArray()[i].energy;
	    auto A = hit.GetMCSimPointArray()[i].A;
	    auto v2 = 2*E/(A*931.5);
	    if(i == 0)
	       Z1 = hit.GetMCSimPointArray()[i].Z;
	    //std::cout << E << " " << A << " " << v2 << std::endl;
	    hVInitial->Fill(TMath::Sqrt(v2));
	 }

      }
      for(int bin = 0; bin < dEdx[i]->GetNbinsX(); ++bin)
	 dEdx[i]->SetBinError(bin, TMath::Sqrt(dEdx[i]->GetBinContent(bin)));
   }

   bool lastBinHadData = false;
   double lastE0 = 0, lastE1 = 0;
   std::vector<double> zVec;
   int numRecorded = 0;
   double sumZ = 0;
   for(int bin = 0; bin < dEdx[0]->GetNbinsX(); ++bin)
   {
      auto E0 = dEdx[0]->GetBinContent(bin);
      auto E1 = dEdx[1]->GetBinContent(bin);
      
      bool thisBinHasData = E0 > 2000 && E1 > 2000;
      if(lastBinHadData && thisBinHasData && numRecorded < numPoints)
      {
	 if (TMath::Abs(lastE0-E0)/(E0+lastE0) > .5 || TMath::Abs(lastE1-E1)/(E1+lastE1) > .5)
	    break;
	 auto R = TMath::Abs((lastE0-lastE1)/(lastE0+lastE1));
	 hdEratio->Fill(R);
	 hdEratio->Fill(-R);

	 auto z1 = calcZ1(R);
	 hZDistro->Fill(z1);
	 hZDistro->Fill(Zcn-z1);

	 hZDiff->Fill(z1 - Z1);
	 hZDiff->Fill(Z1 - z1);

	 numRecorded++;
	 if(z1 < Zcn/2.)
	    sumZ += z1;
	 else
	    sumZ += Zcn-z1;
	 
      }
      
      lastBinHadData = thisBinHasData;
      lastE0 = E0;
      lastE1 = E1;
   }
   std:: cout << sumZ << " " << sumZ/numRecorded << std::endl;
   hZAvg->Fill(sumZ/numRecorded);
   hZAvg->Fill(Zcn - sumZ/numRecorded);
   
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
   for(int i = 0; i < 2; ++i)
      dEdx[i]->Reset();
   
}
void createHistograms(double binWidth)
{
   double maxLength = TMath::Sqrt(TMath::Sq(1000) + TMath::Sq(25));
   int nBins = TMath::CeilNint(maxLength/binWidth);

   std::cout << "Creating dE/dx curve over " << nBins << " bins" << std::endl;
   for(int i = 0; i < 2; ++i)
      if(dEdx[i] != nullptr)
	 delete dEdx[i];
   if(dEdxStack != nullptr)
      delete dEdxStack;
   dEdxStack = new THStack("hs", "Stacked dE/dx curves");   
   
   dEdx[0] = new TH1F("dEdx1", "dEdX Frag 1", nBins, 0, maxLength);
   dEdx[0]->SetMarkerStyle(21);
   dEdx[0]->SetMarkerColor(kRed);
   dEdx[1] = new TH1F("dEdx2", "dEdX Frag 2", nBins, 0, maxLength);
   dEdx[1]->SetMarkerStyle(21);
   dEdx[1]->SetMarkerColor(kGreen);

   dEdxStack->Add(dEdx[0]);
   dEdxStack->Add(dEdx[1]);
   
   
}

double getHitDistanceFromVertex(const AtHit& hit)
{
   auto p = hit.GetPosition();
   auto position = XYZVector(p.x(), p.y(), p.z());
   auto diff = position - vertex;
   return TMath::Sqrt(diff.Mag2());
}

double setLineVectorsAndVertex()
{
   lineStart.clear();
   lineStep.clear();
   for( auto &track : ransacPtr->GetTrackCand() )
   {
      lineStart.emplace_back(XYZVector(track.GetFitPar()[0], track.GetFitPar()[2], 0));
      lineStep.emplace_back(XYZVector(track.GetFitPar()[1], track.GetFitPar()[3], 1));
   }
   vertex = calcualteVetrex();

   auto dot = lineStep[0].Unit().Dot(lineStep[1].Unit());
   return TMath::ACos(dot) * TMath::RadToDeg();
}

XYZVector calcualteVetrex()
{
   auto n = lineStep[0].Cross(lineStep[1]);
   auto n0 = lineStep[0].Cross(n);
   auto n1 = lineStep[1].Cross(n);

   auto c0 = lineStart[0] + (lineStart[1] - lineStart[0]).Dot(n1)/lineStep[0].Dot(n1) * lineStep[0];
   auto c1 = lineStart[1] + (lineStart[0] - lineStart[1]).Dot(n0)/lineStep[1].Dot(n0) * lineStep[1];

   return (c0 + c1)/2.;
}
