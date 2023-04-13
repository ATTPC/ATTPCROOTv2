#include "AtMCFission.h"

#include "AtPatternEvent.h"
#include "AtPatternLine.h"
#include "AtSimpleSimulation.h"
#include "AtUniformDistribution.h"

#include <Math/Vector4D.h>

using namespace MCFitter;
using namespace AtPatterns;
using Polar3D = ROOT::Math::Polar3DVector;
using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

void AtMCFission::SetCN(int Z, int A)
{
   fZcn = Z;
   fAcn = A;
}

AtMCFission::XYZPoint
AtMCFission::calcualteVetrex(const std::vector<XYZVector> &lineStart, const std::vector<XYZVector> &lineStep)
{
   auto n = lineStep[0].Cross(lineStep[1]);
   auto n0 = lineStep[0].Cross(n);
   auto n1 = lineStep[1].Cross(n);

   auto c0 = lineStart[0] + (lineStart[1] - lineStart[0]).Dot(n1) / lineStep[0].Dot(n1) * lineStep[0];
   auto c1 = lineStart[1] + (lineStart[0] - lineStart[1]).Dot(n0) / lineStep[1].Dot(n0) * lineStep[1];

   return XYZPoint((c0 + c1) / 2.);
}

void AtMCFission::CreateParamDistros()

{
   fParameters["th0"] = std::make_shared<AtUniformDistribution>(0, 0);
   fParameters["ph0"] = std::make_shared<AtUniformDistribution>(0, 0);
   fParameters["th1"] = std::make_shared<AtUniformDistribution>(0, 0);
   fParameters["ph1"] = std::make_shared<AtUniformDistribution>(0, 0);
   fParameters["vX"] = std::make_shared<AtUniformDistribution>(0, 0);
   fParameters["vY"] = std::make_shared<AtUniformDistribution>(0, 0);
   fParameters["vZ"] = std::make_shared<AtUniformDistribution>(0, 0);
   fParameters["Z"] = std::make_shared<AtUniformDistribution>(0, 0);
}

void AtMCFission::SetParamsFromEvent(const AtPatternEvent &event)
{
   const auto &tracks = event.GetTrackCand();

   if (tracks.size() < 2) {
      LOG(error) << "Not enough tracks";
      return;
   }
   std::vector<XYZVector> start, dirs;

   //*** Fragment 1 ****/
   auto pat = dynamic_cast<const AtPatternLine *>(tracks[0].GetPattern());
   auto dir = pat->GetDirection().Unit();
   dirs.push_back(dir);
   if (dir.Z() < 0)
      dir.SetZ(-dir.Z());

   fParameters["th0"]->SetMean(dir.Theta());
   fParameters["ph0"]->SetMean(dir.Phi());
   LOG(info) << "Parameters: " << dir.Theta() * TMath::RadToDeg() << " " << dir.Phi() * TMath::RadToDeg();
   start.emplace_back(pat->GetPoint());

   //*** Fragment 2 ****/
   pat = dynamic_cast<const AtPatternLine *>(tracks[1].GetPattern());
   dir = pat->GetDirection().Unit();
   dirs.push_back(dir);
   if (dir.Z() < 0)
      dir.SetZ(-dir.Z());

   fParameters["th1"]->SetMean(dir.Theta());
   fParameters["ph1"]->SetMean(dir.Phi());
   start.emplace_back(pat->GetPoint());
   LOG(info) << "Parameters: " << dir.Theta() * TMath::RadToDeg() << " " << dir.Phi() * TMath::RadToDeg();

   auto vertex = calcualteVetrex(start, dirs);

   fParameters["vX"]->SetMean(vertex.X());
   fParameters["vY"]->SetMean(vertex.Y());
   fParameters["vZ"]->SetMean(1000 - vertex.Z());

   fParameters["Z"]->SetMean(fZcn / 2);
}

double AtMCFission::ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID)
{
   return 1;
}

void AtMCFission::SimulateEvent()
{
   fSim->NewEvent();

   XYZPoint vertex(fParameters["vX"]->Sample(), fParameters["vY"]->Sample(), fParameters["vZ"]->Sample());
   LOG(info) << "Setting vertex:  " << vertex;

   //**** Fragment 1 *****//
   int Z = fParameters["Z"]->Sample();
   int A = fAcn * (double)Z / fZcn;
   double KE = 35; // MeV/A

   Polar3D momDir = Polar3D(1, fParameters["th0"]->Sample(), fParameters["ph0"]->Sample()).Unit();
   LOG(info) << "Setting dir 1: " << momDir.Theta() * TMath::RadToDeg() << " " << momDir.Phi() * TMath::RadToDeg();

   double m = A * 931.4936; // Mev
   double E = m + 35 * A;
   double p = sqrt(E * E - m * m);
   PxPyPzEVector mom(momDir.X() * p, momDir.Y() * p, momDir.Z() * p, E);
   fSim->SimulateParticle(Z, A, vertex, mom);

   //**** Fragment 2 *******//
   momDir = Polar3D(1, fParameters["th1"]->Sample(), fParameters["ph1"]->Sample()).Unit();
   LOG(info) << "Setting dir 2: " << momDir.Theta() * TMath::RadToDeg() << " " << momDir.Phi() * TMath::RadToDeg();
   m = A * 931.4936; // Mev
   E = m + 35 * A;
   p = sqrt(E * E - m * m);
   mom = PxPyPzEVector(momDir.X() * p, momDir.Y() * p, momDir.Z() * p, E);
   fSim->SimulateParticle(fZcn - Z, fAcn - A, vertex, mom);
}
