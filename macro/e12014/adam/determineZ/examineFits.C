
#include  "../helper.h"
#include "TMultiGraph.h"
#include "THStack.h"
#include "Math/Vector3D.h"


/*** Forward declares ***/
using XYZVector = ROOT::Math::XYZVector;
using XYZPoint = ROOT::Math::XYZPoint;
void setLine(AtTrack &track);
void setLineRansac(AtTrack &track);
double distanceToLine(const AtHit &hit);
bool sortHitZ(const AtHit &lhs, const AtHit &rhs) { return lhs.GetPosition().Z() < rhs.GetPosition().Z();}
XYZVector m;
XYZPoint b;
//Line(t)  = m*t + b

void examineFits( TString fileName =
		 "/mnt/simulations/attpcroot/adam/ATTPCROOTv2/macro/e12014/simulation/eventGenerator/sym/output_digi.root",
		 TString rawEventName = "AtRawEvent", TString eventName = "AtEventH")
{
   loadRun(fileName, rawEventName, eventName);
}

void examineEvent(int eventNumber)
{
   loadEvent(eventNumber);
   
   auto tracks = ransacPtr->GetTrackCand(); // type vector<AtTrack>
   if (tracks.size() != 2)
   {
      std::cout << "Event " << eventNumber << " does not have 2 tracks. Can't construct dE/dx" << std::endl;
      return;
   }
   
   
   for(auto &track : tracks)
   {
      std::cout << "Running new track: " << std::endl;
      setLine(track);
      setLineRansac(track);
      
      auto hitArray = track.GetHitArray();
      std::sort(hitArray.begin(), hitArray.end(), sortHitZ);
      
      for(auto &hit : hitArray)
	 std::cout << hit.GetPosition() << " " << distanceToLine(hit) << std::endl;
   }
   
}
void examineHits(int eventNumber)
{
   loadEvent(eventNumber);
   for(int i = 0; i < eventPtr->GetHitArray().size(); ++i)
      std::cout << i << " " << eventPtr->GetHitArray()[i].GetHitID() << std::endl;
}

void setLine(AtTrack &track)
{
   b = XYZVector(track.GetFitPar()[0], track.GetFitPar()[2], 0);
   m = XYZVector(track.GetFitPar()[1], track.GetFitPar()[3], 1000);

   std::cout << "Setting line to: " << b << " + t * " << m << std::endl;
}
void setLineRansac(AtTrack &track)
{
   b = XYZVector(track.GetRANSACCoeff()[0], track.GetRANSACCoeff()[1], track.GetRANSACCoeff()[2]);
   m = XYZVector(track.GetRANSACCoeff()[3], track.GetRANSACCoeff()[4], track.GetRANSACCoeff()[5]);

   std::cout << "Setting line to: " << b << " + t * " << m << std::endl;
}

double distanceToLine(const AtHit &hit)
{
   auto hitLoc = hit.GetPosition();
   auto hitToLine = b - hitLoc;
   return TMath::Sqrt(hitToLine.Cross(m.Unit()).Mag2());
}
