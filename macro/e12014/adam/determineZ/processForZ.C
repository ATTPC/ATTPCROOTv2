/* Program for processing raw data from TPC into observables
 * used to reconstruct the charge of the fission fragments
 *
 * Adam Anthony 2/23/22
 */

#include "../helper.h"
#include "AtTrack.h"

#include "Math/Vector3D.h"
#include "Math/Point3D.h"
#include "TTree.h"
#include "TFile.h"

/*** Forward declares ***/
using XYZVector = ROOT::Math::XYZVector;
using XYZPoint = ROOT::Math::XYZPoint;
void createOutputFileAndTree(TString fileName, double binWidth);
void processEventAndFillTree(Double_t integrationRadius);
void setAngleAndVertex();
bool setTracks();
void setdEdX();
void setdEdXRatios();
void setSimulationData();

XYZPoint calcualteVetrex(const std::vector<XYZVector> &lineStart, const std::vector<XYZVector> &lineStep);
double getHitDistanceFromVertex(const AtHit &hit);
double getHitDistanceFromVertexAlongZ(const AtHit &hit);

std::array<AtTrack, 2> tracks;

// Branches to fill in tree always
XYZPoint vertex(0, 0, 0);
Double_t angle = 0;
Int_t Zcn = 84;
Double_t intRadius = 0;
std::vector<std::vector<double>> charge;
std::vector<std::vector<double>> chargeIntegral;
std::vector<std::vector<double>> dVertex;
std::vector<std::vector<double>> dVertexZ;

// Branches to fill in tree if simulation
Int_t Z[2] = {0, 0};
Int_t A[2] = {0, 0};
Double E[2] = {0, 0};
Double_t vInitial[2] = {0, 0};
Double_t Rv = 2;
std::vector<XYZVector> pInitial;

TFile *outFile = nullptr;
TTree *outTree = nullptr;

/* Routine to taking raw TPC data and constructing usefule observables

Input:
fileName:
binWidth: in mm for dE/dx curves

optional:
rawEventName:
eventName:

Output:
output.root with a TTree of observables

*/
void mapFile(TString inFileName, TString outFileName, double integrationRadius, Long64_t maxEntry = TTree::kMaxEntries,
             TString rawEventName = "AtRawEvent", TString eventName = "AtEventH")
{
   loadRun(inFileName, rawEventName, eventName);
   createOutputFileAndTree(outFileName, integrationRadius);

   while (nextEvent() && reader->GetCurrentEntry() < maxEntry)
      processEventAndFillTree(integrationRadius);

   outTree->Write();
   delete outTree;
}

void setSimulationData()
{
   for (int i = 0; i < 2; ++i) {

      auto initialPoint = ransacPtr->GetTrackCand()[i].GetHitArray()->at(0).GetMCSimPointArray()[0];
      if (initialPoint == nullptr)
         continue;

      E[i] = initialPoint.energy;
      A[i] = initialPoint.A;
      Z[i] = initialPoint.Z;
      vInitial[i] = TMath::Sqrt(2 * E[i] / (A[i] * 931.5));
   }
}

void processEventAndFillTree(Double_t integrationRadius)
{
   if (!setTracks()) {
      std::cout << "skipping event..." << std::endl;
      return;
   }

   setAngleAndVertex();
   setdEdX(integrationRadius);
   setdEdXRatios();
   setSimulationData();

   outTree->Fill();
}

void createOutputFileAndTree(TString fileName, double intRadius)
{
   if (outFile != nullptr)
      delete outFile;
   if (outTree != nullptr)
      delete outTree;

   outFile = new TFile(fileName, "RECREATE");
   outTree = new TTree("zObs", "Obervables for Z determination of fisison fragments");

   // Initialize branches
   for (int i = 0; i < 2; ++i) {
      charge.push_back({});
      chargeIntegral.push_back({});
      dVertex.push_back({});
      dVertexZ.push_back({});
   }

   // Reconstructed info
   outTree->Branch("vertex", &vertex);
   outTree->Branch("angle", &angle);
   outTree->Branch("zcn", &Zcn);
   outTree->Branch("Q", &charge);
   outTree->Branch("QInt", &chargeIntegral);
   outTree->Branch("radius", &intRadius);
   outTree->Branch("dVertex", &dVertex);
   outTree->Branch("dVertexZ", &dVertexZ);

   // Simulated info
   outTree->Branch("Z", &Z, "Z[2]/I");
   outTree->Branch("A", &A, "A[2]/I");
   outTree->Branch("vInitial", &vInitial, "vInital[2]/D");
   outTree->Branch("vRatio", &Rv);
   outTree->Branch("pInitial", &pInitial);

   std::cout << "Created tree in file: " << fileName << std::endl;
}

void setdEdX(Double_t integrationRadius)
{
   for (int i = 0; i < 2; ++i) {
      charge.at(i).clear();
      chargeIntegral.at(i).clear();
      dVertex.at(i).clear();
      dVertexZ.at(i).clear();

      for (auto &hit : *(tracks[i].GetHitArray())) {

         charge[i].push_back(hit.GetCharge());
         dVertex[i].push_back(getHitDistanceFromVertex(hit));
         dVertexZ[i].push_back(getHitDistanceFromVertexAlongZ(hit));

         chargeIntegral[i].push_back(0);
         for (const auto &hit2 : *(tracks[i].GetHitArray())) {
         }
      }
   }
}

void setAngleAndVertex()
{
   std::vector<XYZVector> lineStart;
   std::vector<XYZVector> lineStep; // each step is 1 mm in z

   for (auto &track : tracks) {
      lineStart.emplace_back(XYZVector(track.GetFitPar()[0], track.GetFitPar()[2], 0));
      lineStep.emplace_back(XYZVector(track.GetFitPar()[1], track.GetFitPar()[3], 1));
   }
   vertex = calcualteVetrex(lineStart, lineStep);

   auto dot = lineStep[0].Unit().Dot(lineStep[1].Unit());
   angle = TMath::ACos(dot) * TMath::RadToDeg();
}

bool setTracks()
{
   auto trackIn = ransacPtr->GetTrackCand(); // type vector<AtTrack>
   if (trackIn.size() != 2)
      return false;

   tracks[0] = trackIn[0];
   tracks[1] = trackIn[1];

   return true;
}

XYZPoint calcualteVetrex(const std::vector<XYZVector> &lineStart, const std::vector<XYZVector> &lineStep)
{
   auto n = lineStep[0].Cross(lineStep[1]);
   auto n0 = lineStep[0].Cross(n);
   auto n1 = lineStep[1].Cross(n);

   auto c0 = lineStart[0] + (lineStart[1] - lineStart[0]).Dot(n1) / lineStep[0].Dot(n1) * lineStep[0];
   auto c1 = lineStart[1] + (lineStart[0] - lineStart[1]).Dot(n0) / lineStep[1].Dot(n0) * lineStep[1];

   return XYZPoint((c0 + c1) / 2.);
}

double getHitDistanceFromVertex(const AtHit &hit)
{
   auto p = hit.GetPosition();
   auto position = XYZPoint(p.x(), p.y(), p.z());
   auto diff = position - vertex;
   return TMath::Sqrt(diff.Mag2());
}

double getHitDistanceFromVertexAlongZ(const AtHit &hit)
{
   auto p = hit.GetPosition();
   auto diff = p.z() - vertex.z();
   return TMath::Abs(diff);
}
