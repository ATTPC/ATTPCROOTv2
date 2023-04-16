#include "AtMCFission.h"
// IWYU pragma: no_include <ext/alloc_traits.h>

#include "AtEvent.h"
#include "AtHit.h"                   // for AtHit, AtHit::XYZPoint, AtHit...
#include "AtParameterDistribution.h" // for AtParameterDistribution, MCFi...
#include "AtPattern.h"               // for AtPattern, AtPatterns
#include "AtPatternEvent.h"
#include "AtPatternLine.h"
#include "AtSimpleSimulation.h"
#include "AtStudentDistribution.h"
#include "AtUniformDistribution.h"
#include "AtVectorUtil.h"

#include <FairLogger.h> // for Logger, LOG

#include <Math/AxisAngle.h> // for AxisAngle
#include <Math/Vector4D.h>
#include <Math/VectorUtil.h> // for Angle
#include <TClonesArray.h>    // for TClonesArray
#include <TMath.h>           // for RadToDeg, C, DegToRad, Pi
#include <TObject.h>         // for TObject
#include <TRandom.h>

#include <algorithm> // for find_if
#include <iostream>  // for endl, cout, ostream
#include <map>       // for map, map<>::mapped_type
#include <memory>    // for make_shared, __shared_ptr_access
#include <string>    // for string
#include <vector>    // for vector, allocator

using namespace MCFitter;
using namespace AtPatterns;
using Polar3D = ROOT::Math::Polar3DVector;
using XYZPoint = ROOT::Math::XYZPoint;
using XYZVector = ROOT::Math::XYZVector;
using PxPyPzEVector = ROOT::Math::PxPyPzEVector;

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

   // These are the angle to sample w.r.t. the nominal beam axis
   fParameters["thBeam"] = std::make_shared<AtStudentDistribution>(0, 1 * TMath::DegToRad());
   fParameters["phBeam"] = std::make_shared<AtStudentDistribution>(0, TMath::Pi());
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
   LOG(debug) << "Parameters: " << dir.Theta() * TMath::RadToDeg() << " " << dir.Phi() * TMath::RadToDeg();
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
   LOG(debug) << "Parameters: " << dir.Theta() * TMath::RadToDeg() << " " << dir.Phi() * TMath::RadToDeg();

   auto vertex = AtTools::GetIntersection(start, dirs);

   fParameters["vX"]->SetMean(vertex.X());
   fParameters["vY"]->SetMean(vertex.Y());
   fParameters["vZ"]->SetMean(1000 - vertex.Z());

   fParameters["Z"]->SetMean(fCN.Z / 2);
}

double AtMCFission::ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID)
{
   return 1;
}

double AtMCFission::ObjectivePosition(AtEvent &expEvent, int SimEventID)
{
   AtEvent &event = *dynamic_cast<AtEvent *>(fEventArray.At(SimEventID));

   // Sort the hits by pad number
   event.SortHitArray();
   expEvent.SortHitArray();
   auto lastHit = event.GetHits().begin();

   double chi2 = 0;
   // Get every pad that was in the event
   for (auto &hit : expEvent.GetHits()) {
      auto padNum = hit->GetPadNum();

      // Look for pad in simulated event
      auto hitIt = std::find_if(lastHit, event.GetHits().end(),
                                [padNum](const AtEvent::HitPtr &hit2) { return hit2->GetPadNum() == padNum; });
      if (hitIt == event.GetHits().end()) {
         chi2++; // This is the integral of the normalized gaussian that we have nothing to compare to
         LOG(info) << "Did not find pad " << padNum << "in the simulated event.";
         continue;
      }

      // Update the chi2 with this hit

      lastHit = hitIt;
      double diff = ObjectivePosition(hit->GetPosition().Z(), hit->GetPositionSigma().Z(),
                                      lastHit->get()->GetPosition().Z(), lastHit->get()->GetPositionSigma().Z());
      LOG(info) << "Found " << padNum << "in the simulated event. Chi2 between pads is " << diff;
      chi2 += diff;
   }
   return chi2;
}
/**
 * Returns \Integral_{-inf}^{inf} (G[uO,sO,x] - G[uE,sE,x])^2/G[uE,sE,x] dx
 * Assuming the Gaussain functions are normalized.
 *
 * The experimental is the expected, the observed is our simulated event
 */
double AtMCFission::ObjectivePosition(double uE, double sE, double uO, double sO)
{
   double num = sE * sE * std::exp(((uE - uO) * (uE - uO)) / (2 * sE * sE - sO * sO));
   double denom = sO * std::sqrt(2 * sE * sE - sO * sO);
   return num / denom - 1;
}
XYZPoint AtMCFission::SampleVertex()
{
   return {fParameters["vX"]->Sample(), fParameters["vY"]->Sample(), fParameters["vZ"]->Sample()};
}

std::array<Ion, 2> AtMCFission::SampleFragmentSpecies()
{
   int Z1 = std::round(fParameters["Z"]->Sample());
   int A1 = std::round(fCN.A * (double)Z1 / fCN.Z);

   if (gRandom->Integer(2))
      return {Ion{Z1, A1}, Ion{fCN.Z - Z1, fCN.A - A1}};
   return {Ion{fCN.Z - Z1, fCN.A - A1}, Ion{Z1, A1}};
}
/**
 * Get gamma for fragment 1 in a system decaying into two fragments with total KE
 * Units are SR (c=1).
 */
double AtMCFission::GetGamma(double KE, double m1, double m2)
{
   double num = KE * KE + 2 * (m1 + m2) * (KE + m1);
   double denom = 2 * m1 * (KE + m1 + m2);
   return num / denom;
}

/**
 * Get velocity (cm/ns) of a particle with gamma.
 */
double AtMCFission::GetVelocity(double gamma)
{
   return GetBeta(gamma) * TMath::C() * 1e-7;
}

/**
 * Get velocity (SR units, c=1) of a particle with gamma.
 */
double AtMCFission::GetBeta(double gamma)
{
   return std::sqrt(gamma * gamma - 1) / gamma;
}

/**
 * Get the relativistic momentum of a particle will mass (MeV)
 */
double AtMCFission::GetRelMom(double gamma, double mass)
{
   return std::sqrt(gamma * gamma - 1) * mass;
}

/**
 * Get the mass in MeV of a fragment of mass in amu (or A)
 */
double AtMCFission::AtoE(double Amu)
{
   return Amu * 931.5;
}

XYZVector AtMCFission::SampleBeamDir()
{
   // Generate the beam
   Polar3D beamDir(1, fParameters["thBeam"]->Sample(), fParameters["phBeam"]->Sample());
   LOG(debug) << "Sampled beam direction (deviation from nominal)"
              << ROOT::Math::VectorUtil::Angle(XYZVector(0, 0, 1), beamDir) * TMath::RadToDeg() << " "
              << beamDir.Theta() * TMath::RadToDeg();

   // Rotate the beam vector so our deviation is from the nominal beam axis
   auto rot = AtTools::GetRotationToZ(fNominalBeamDir);
   auto beamInLabFrame = XYZVector(rot.Inverse()(beamDir));
   LOG(debug) << "Sampled beam direction in lab frame: " << beamInLabFrame * 1000 / beamInLabFrame.Z();
   LOG(info) << " Sampled beam deviates from nominal by "
             << ROOT::Math::VectorUtil::Angle(beamInLabFrame, fNominalBeamDir) * TMath::RadToDeg() << " def.";

   return beamInLabFrame.Unit();
}
std::array<XYZVector, 2> AtMCFission::SampleMomDir()
{
   XYZVector mom1(Polar3D(1, fParameters["th0"]->Sample(), fParameters["ph0"]->Sample()).Unit());
   XYZVector mom2(Polar3D(1, fParameters["th1"]->Sample(), fParameters["ph1"]->Sample()).Unit());
   return {mom1, mom2};
}

/**
 * Set the magnitude of the FF momenta.
 * The transverse momentum of the fission fragment in the lab frame is the same as in the CoM frame
 * because of our assumtion on the decay angle. By transvse I mean transverse w.r.t. the beam axis.
 * We can calculate the magnitude of p for a FF by comparing the angle between the beam and FF.
 */
void AtMCFission::SetMomMagnitude(XYZVector beamDir, std::array<XYZVector, 2> &mom, const std::array<double, 2> &pTrans)
{
   for (int i = 0; i < mom.size(); ++i) {
      auto angle = ROOT::Math::VectorUtil::Angle(beamDir, mom[i]);
      auto p = pTrans[i] / std::sin(angle);
      mom[i] *= p;
   }
}

void AtMCFission::SimulateEvent()
{
   fSim->NewEvent();

   XYZPoint vertex = SampleVertex();
   std::cout << std::endl;
   LOG(info) << "Setting vertex:  " << vertex;

   auto fragID = SampleFragmentSpecies();
   LOG(info) << "Setting fragment 1: " << fragID[0].Z << " " << fragID[0].A;
   LOG(info) << "Setting fragment 2: " << fragID[1].Z << " " << fragID[1].A;

   // We have the masses, now calculate the momentum of these two fragments in the CoM frame. We will
   // fudge and say the mass of a fragment is A amu.
   double KE = violaEn(fCN.A, fCN.Z);
   double p1 = GetRelMom(GetGamma(KE, AtoE(fragID[0].A), AtoE(fragID[1].A)), AtoE(fragID[0].A));
   double p2 = p1; // GetRelMom(GetGamma(KE, AtoE(fragID[1].A), AtoE(fragID[0].A)), AtoE(fragID[1].A));
   LOG(info) << "Total KE : " << KE << "MeV. P1 = " << p1 << " MeV/c. P2 = " << p2 << " MeV/c.";

   // Assume the decay happens 90 deg w.r.t the beam axis. Then the transverse momentum is the same
   // in both the lab and CoM frame.

   // Get the beam direction in the lab frame and the FF momentum direction in the lab frame
   auto beamDir = SampleBeamDir();
   auto mom = SampleMomDir();
   SetMomMagnitude(beamDir, mom, {p1, p2});

   // Now that we have the momentum of the FF, generate the 4 vectors
   std::vector<XYZEVector> ffMom(2);
   ffMom[0] = Get4Vector(mom[0], AtoE(fragID[0].A));
   ffMom[1] = Get4Vector(mom[1], AtoE(fragID[1].A));

   // Use conservation of 4 momentum to get the beam momentum
   auto pBeam = ffMom[0] + ffMom[1];

   LOG(info) << "Frag A: " << ffMom[0] << " mass " << EtoA(ffMom[0].M()) << " energy: " << ffMom[0].E() - ffMom[0].M();
   LOG(info) << "Frag B: " << ffMom[1] << " mass " << EtoA(ffMom[1].M()) << " energy: " << ffMom[1].E() - ffMom[1].M();
   LOG(info) << "Beam: " << pBeam << " mass " << EtoA(pBeam.M()) << " energy: " << pBeam.E() - pBeam.M();
   for (int i = 0; i < 2; ++i)
      fSim->SimulateParticle(fragID[i].Z, fragID[i].A, vertex, ffMom[i]);
}
