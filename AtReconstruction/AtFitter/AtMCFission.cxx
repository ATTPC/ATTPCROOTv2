#include "AtMCFission.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtEvent.h"
#include "AtFissionEvent.h"
#include "AtHit.h" // for AtHit, AtHit::XYZPoint, AtHit...
#include "AtMap.h"
#include "AtParameterDistribution.h" // for AtParameterDistribution, MCFi...
#include "AtPattern.h"               // for AtPattern, AtPatterns
#include "AtPatternEvent.h"
#include "AtPatternLine.h"
#include "AtPulse.h"
#include "AtRadialChargeModel.h"
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
   fParameters["lambda"] = std::make_shared<AtUniformDistribution>(0, 0);
}

void AtMCFission::SetParamDistributions(const AtPatternEvent &event)
{

   const auto &fissionEvent = dynamic_cast<const AtFissionEvent &>(event);

   auto pat = fissionEvent.GetYPattern();
   std::vector<XYZVector> start, dirs;

   //*** Fragment 1 ****/
   auto dir = pat->GetFragmentDirection(0).Unit();
   if (dir.Z() < 0)
      dir.SetZ(-dir.Z());
   fParameters["th0"]->SetMean(dir.Theta());
   fParameters["ph0"]->SetMean(dir.Phi());
   LOG(debug) << "Parameters: " << dir.Theta() * TMath::RadToDeg() << " " << dir.Phi() * TMath::RadToDeg();

   /*** Fragment 2 ****/
   dir = dir = pat->GetFragmentDirection(1).Unit();
   if (dir.Z() < 0)
      dir.SetZ(-dir.Z());
   fParameters["th1"]->SetMean(dir.Theta());
   fParameters["ph1"]->SetMean(dir.Phi());
   LOG(debug) << "Parameters: " << dir.Theta() * TMath::RadToDeg() << " " << dir.Phi() * TMath::RadToDeg();

   auto vertex = pat->GetVertex();

   fParameters["vX"]->SetMean(vertex.X());
   fParameters["vY"]->SetMean(vertex.Y());
   fParameters["vZ"]->SetMean(1000 - vertex.Z());

   fParameters["Z"]->SetMean(fCN.Z / 2);
   fParameters["lambda"]->SetMean(fissionEvent.GetLambda());
}

double AtMCFission::ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID)
{
   // Make sure we were passed the right event type

   auto expFission = dynamic_cast<const AtFissionEvent &>(expEvent);

   return ObjectivePosition(expFission, SimEventID);
}

double AtMCFission::ObjectivePosition(const AtFissionEvent &expEvent, int SimEventID)
{
   AtEvent &event = *dynamic_cast<AtEvent *>(fEventArray.At(SimEventID));

   // Sort the hits by pad number
   event.SortHitArray();
   auto fragHits = expEvent.GetFragHits();

   std::sort(fragHits.begin(), fragHits.end(),
             [](const AtHit *a, const AtHit *b) { return a->GetPadNum() < b->GetPadNum(); });

   auto lastHit = event.GetHits().begin();

   auto map = fPulse->GetMap();
   double chi2 = 0;

   // Get every pad that was in the experimental event (assumes that each pad was only used once)
   for (auto hit : fragHits) {
      auto padNum = hit->GetPadNum();

      // Only keep hits where the pad is not inhibited in some way
      if (fMap->IsInhibited(padNum) != AtMap::InhibitType::kNone)
         continue;

      // Look for pad in simulated event
      auto hitIt = std::find_if(lastHit, event.GetHits().end(),
                                [padNum](const AtEvent::HitPtr &hit2) { return hit2->GetPadNum() == padNum; });
      if (hitIt == event.GetHits().end()) {
         chi2++; // This is the integral of the normalized gaussian that we have nothing to compare to
         LOG(debug) << "Did not find pad " << padNum << "in the simulated event.";
         continue;
      }

      // Update the chi2 with this hit

      lastHit = hitIt;
      double diff = ObjectivePosition(hit->GetPosition().Z(), hit->GetPositionSigma().Z(),
                                      lastHit->get()->GetPosition().Z(), lastHit->get()->GetPositionSigma().Z());
      LOG(debug) << "Found " << padNum << "in the simulated event. Chi2 between pads is " << diff;
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

XYZPoint AtMCFission::GetVertex(AtMCResult &res)
{

   return {res.fParameters["vX"], res.fParameters["vY"], res.fParameters["vZ"]};
}

std::array<Ion, 2> AtMCFission::GetFragmentSpecies(AtMCResult &res, const Ion &CN)
{
   int Z1 = res.fParameters["Z0"];
   int A1 = res.fParameters["A0"];

   return {Ion{Z1, A1}, Ion{CN.Z - Z1, CN.A - A1}};
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

XYZVector AtMCFission::GetBeamDir(AtMCResult &res)
{
   // Sample the deviation of the beam away from the nominal beam direction
   Polar3D beamDir(1, res.fParameters["thBeam"], 0);
   LOG(debug) << "Sampled beam direction (deviation from nominal)"
              << ROOT::Math::VectorUtil::Angle(XYZVector(0, 0, 1), beamDir) * TMath::RadToDeg() << " "
              << beamDir.Theta() * TMath::RadToDeg();

   // Rotate the beam vector so our deviation is from the nominal beam axis
   auto rot = AtTools::GetRotationToZ(fNominalBeamDir);
   auto beamInLabFrame = XYZVector(rot.Inverse()(beamDir));
   LOG(debug) << "Sampled beam direction in lab frame: " << beamInLabFrame * 1000 / beamInLabFrame.Z();
   LOG(info) << " Sampled beam deviates from nominal by "
             << ROOT::Math::VectorUtil::Angle(beamInLabFrame, fNominalBeamDir) * TMath::RadToDeg() << " def.";

   // Set the beam direction in the result file as well
   res.fParameters["beamX"] = beamInLabFrame.Unit().X();
   res.fParameters["beamY"] = beamInLabFrame.Unit().Y();
   res.fParameters["beamZ"] = beamInLabFrame.Unit().Z();
   return beamInLabFrame.Unit();
}

/**
 * Get the direction of the FF in the lab frame.
 */
std::array<XYZVector, 2> AtMCFission::GetMomDirLab(AtMCResult &res)
{
   XYZVector mom1(Polar3D(1, res.fParameters["th0"], res.fParameters["ph0"]).Unit());
   XYZVector mom2(Polar3D(1, res.fParameters["th1"], res.fParameters["ph1"]).Unit());
   return {mom1, mom2};
}

AtMCResult AtMCFission::DefineEvent()
{
   LOG(info) << "Defining the event.";

   AtMCResult result = AtMCFitter::DefineEvent();

   // Sample variations away from nominal FF momenta directions and vertex location
   int Z1 = std::round(result.fParameters["Z"]);
   int A1 = std::round(fCN.A * (double)Z1 / fCN.Z);
   if (gRandom->Integer(2)) {
      Z1 = fCN.Z - Z1;
      A1 = fCN.A - A1;
   }
   result.fParameters["Z0"] = Z1;
   result.fParameters["A0"] = A1;

   result.fParameters["thCM"] = 90 * TMath::DegToRad(); // Decay angle in CoM frame
   return result;
}

/**
 * Set the magnitude of the FF momenta.
 * The transverse momentum of the fission fragment in the lab frame is the same as in the CoM frame
 * because of our assumtion on the decay angle. By transvse I mean transverse w.r.t. the beam axis.
 * We can calculate the magnitude of p for a FF by comparing the angle between the beam and FF.
 */
void AtMCFission::SetMomMagnitude(XYZVector beamDir, std::array<XYZVector, 2> &mom, double pTrans)
{
   for (int i = 0; i < mom.size(); ++i) {
      auto angle = ROOT::Math::VectorUtil::Angle(beamDir, mom[i]);
      auto p = pTrans / std::sin(angle);
      mom[i] *= p;
   }
}

TClonesArray AtMCFission::SimulateEvent(AtMCResult def)
{
   fSim->NewEvent();
   auto scModel = dynamic_cast<AtRadialChargeModel *>(fSim->GetSpaceChargeModel().get());
   if (scModel) {
      scModel->SetDistortionField(AtLineChargeZDep(def.fParameters["lambda"]));
      LOG(info) << "Setting Lambda: " << def.fParameters["lambda"];
   } else
      LOG(info) << "No space charge to apply.";

   auto vertex = GetVertex(def);
   LOG(info) << "Setting vertex: " << vertex;

   auto fragID = GetFragmentSpecies(def, fCN);
   LOG(info) << "Setting fragment 1: " << fragID[0].Z << " " << fragID[0].A;
   LOG(info) << "Setting fragment 2: " << fragID[1].Z << " " << fragID[1].A;

   // We have the masses, now calculate the momentum of these two fragments in the CoM frame. We will
   // fudge and say the mass of a fragment is A amu.
   double KE = violaEn(fCN.A, fCN.Z);
   double gamma1 = GetGamma(KE, AtoE(fragID[0].A), AtoE(fragID[1].A));
   double p1 = GetRelMom(gamma1, AtoE(fragID[0].A));

   // Get the momentum direction for the FF and beam in the lab frame
   auto beamDir = GetBeamDir(def);
   auto mom = GetMomDirLab(def);

   // Set the momentum of the fission fragments assuming that the
   // We need to adjust p1 and p2 to get the transverse momentum component
   p1 *= sin(def.fParameters["thCM"]);
   SetMomMagnitude(beamDir, mom, p1);

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

   return fSim->GetPointsArray();
}
