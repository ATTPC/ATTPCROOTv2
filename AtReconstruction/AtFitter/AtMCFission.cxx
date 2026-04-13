#include "AtMCFission.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtContainerManip.h" // for GetPointerVector
#include "AtE12014.h"
#include "AtEvent.h"
#include "AtFissionEvent.h"
#include "AtHit.h" // for AtHit, AtHit::XYZPoint, AtHit...
#include "AtKinematics.h"
#include "AtLineChargeModel.h"
#include "AtMap.h"
#include "AtParameterDistribution.h" // for AtParameterDistribution, MCFi...
#include "AtPattern.h"               // for AtPattern, AtPatterns
#include "AtPatternEvent.h"
#include "AtPatternY.h" // for AtPatternY, AtPatternY::XYZVector
#include "AtPulse.h"
#include "AtRadialChargeModel.h"
#include "AtSimpleSimulation.h"
#include "AtStudentDistribution.h"
#include "AtUniformDistribution.h"
#include "AtVectorUtil.h"

#include <FairLogger.h> // for Logger, LOG

#include <Math/AxisAngle.h> // for AxisAngle
#include <Math/Functor.h>
#include <Math/Vector4D.h>
#include <Math/VectorUtil.h> // for Angle
#include <TClonesArray.h>    // for TClonesArray
#include <TMath.h>           // for RadToDeg, C, DegToRad, Pi

#include <Fit/FitConfig.h> // for FitConfig
#include <Fit/FitResult.h> // for FitResult
#include <Fit/Fitter.h>
#include <algorithm> // for find_if
#include <cassert>   // for assert
#include <cmath>     // for sqrt, exp, sin, round
#include <limits>    // for numeric_limits
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

void AtMCFission::SetAmp(float amp)
{
   fAmp = amp;
   fFitAmp = false;
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
   fParameters["Z"] = std::make_shared<AtStudentDistribution>(0, 0);

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
   fParameters["Z"]->SetSpread(3);
   fParameters["lambda"]->SetMean(fissionEvent.GetLambda());
}

double AtMCFission::ObjectiveFunction(const AtBaseEvent &expEvent, int SimEventID, AtMCResult &definition)
{
   // Make sure we were passed the right event type
   auto expFission = dynamic_cast<const AtFissionEvent &>(expEvent);
   auto charge = ObjectiveCharge(expFission, SimEventID, definition);
   // auto charge = ObjectiveChargePads(expFission, SimEventID, definition);
   auto pos = ObjectivePositionPads(expFission, SimEventID);
   LOG(info) << "Chi2 Pos: " << pos << " Chi2 Q: " << charge;
   definition.fParameters["ObjQ"] = charge;
   definition.fParameters["ObjPos"] = pos;

   return charge;
}

/**
 * Gets the max charge of a hit in the experimental event that is associated with a FF (Qmax)
 * Fits the scaling parameter, A to minimize the following (which is the objective funtion)
 *
 * Chi2 = 1/N \Sum (Q_exp - A*Q_sim)^2/(0.1*Qmax)^2
 * summing over all good pads in the experimental event.
 */
double AtMCFission::ObjectiveChargePads(const AtFissionEvent &expEvent, int SimEventID, AtMCResult &def)
{
   AtEvent &simEvent = fEventArray.at(SimEventID);
   auto fragHits1 = expEvent.GetFragHits(0);
   auto fragHits2 = expEvent.GetFragHits(1);

   // Get the max charge in the experimental event
   auto comp = [](const AtHit *a, const AtHit *b) { return a->GetCharge() < b->GetCharge(); };

   auto maxQ1 = (*std::max_element(fragHits1.begin(), fragHits1.end(), comp))->GetCharge();
   auto maxQ2 = (*std::max_element(fragHits2.begin(), fragHits2.end(), comp))->GetCharge();

   // Loop through every hit in the experimental event associated with the FF, and create an array
   //  of the corresponding hits in the simulated event
   std::vector<std::vector<double>> expCharge, simCharge;
   expCharge.resize(2);
   simCharge.resize(2);
   E12014::FillHits(expCharge[0], simCharge[0], fragHits1, ContainerManip::GetPointerVector(simEvent.GetHits()),
                    E12014::fSatThreshold);
   E12014::FillHits(expCharge[1], simCharge[1], fragHits2, ContainerManip::GetPointerVector(simEvent.GetHits()),
                    E12014::fSatThreshold);

   // So now that we have our two vectors to compare, run the minimization
   assert(expCharge[0].size() == simCharge[0].size() && expCharge[1].size() == simCharge[1].size());

   auto func = [&expCharge, &simCharge, maxQ1, maxQ2](const double *par) {
      double chi2_1 = 0;
      for (int i = 0; i < expCharge[0].size(); ++i) {
         double chi = (expCharge[0][i] - par[0] * simCharge[0][i]) / (0.1 * maxQ1);
         chi2_1 += chi * chi;
      }
      chi2_1 /= expCharge[0].size();

      double chi2_2 = 0;
      for (int i = 0; i < expCharge[1].size(); ++i) {
         double chi = (expCharge[1][i] - par[0] * simCharge[1][i]) / (0.1 * maxQ2);
         chi2_2 += chi * chi;
      }
      chi2_2 /= expCharge[1].size();

      return chi2_1 + chi2_2;
   };
   auto functor = ROOT::Math::Functor(func, 1);

   std::vector<double> A = {fAmp};
   ROOT::Fit::Fitter fitter;
   fitter.Config().SetMinimizer("Minuit2");
   fitter.SetFCN(functor, A.data());
   bool ok = fitter.FitFCN();
   if (!ok) {
      LOG(error) << "Failed to fit the charge curves. Not adding to objective.";
      return std::numeric_limits<double>::max();
   }
   auto &result = fitter.Result();
   double amp = result.Parameter(0);
   def.fParameters["Amp"] = amp;
   def.fParameters["qChi2"] = result.MinFcnValue();
   return result.MinFcnValue();
}

double AtMCFission::ObjectivePositionPads(const AtFissionEvent &expEvent, int SimEventID)
{
   AtEvent &simEvent = fEventArray.at(SimEventID);
   auto fragHits = expEvent.GetFragHits();

   std::vector<double> exp, sim;
   E12014::FillZPos(exp, sim, fragHits, ContainerManip::GetPointerVector(simEvent.GetHits()), E12014::fSatThreshold);

   assert(exp.size() == sim.size());
   double chi2 = 0;
   for (int i = 0; i < exp.size(); ++i) {

      double chi = (exp[i] - sim[i]) / (1 * 0.320 * .815 * 10); // 1 TB = 320 ns = .32*.815*10 mm
      chi2 += chi * chi;
   }
   return chi2 / exp.size();
}

double AtMCFission::ObjectiveCharge(const AtFissionEvent &expEvent, int SimEventID, AtMCResult &def)
{
   AtEvent &simEvent = fEventArray.at(SimEventID);
   auto fragHits = expEvent.GetFragHits();

   // Get the charge curves for this event
   std::array<std::vector<double>, 2> exp;
   std::array<std::vector<double>, 2> sim;
   for (int i = 0; i < 2; ++i) {
      E12014::FillHitSums(exp[i], sim[i], expEvent.GetFragHits(i), ContainerManip::GetPointerVector(simEvent.GetHits()),
                          E12014::fThreshold, E12014::fSatThreshold, fPar);
   }

   return ObjectiveCharge(exp, sim, def);
}

double
AtMCFission::ObjectiveChargeChi2(const std::vector<double> &exp, const std::vector<double> &sim, const double *par)
{
   if (exp.size() == 0 || exp.size() != sim.size())
      return std::numeric_limits<double>::max();

   double chi2 = 0;
   for (int i = 0; i < exp.size(); ++i)
      chi2 += (exp.at(i) - par[0] * sim.at(i)) * (exp.at(i) - par[0] * sim.at(i)) / exp.at(i);

   chi2 /= exp.size();
   LOG(debug) << "A: " << par[0] << " Chi2: " << chi2;
   return chi2;
}

double
AtMCFission::ObjectiveChargeChi2Norm(const std::vector<double> &exp, const std::vector<double> &sim, const double *par)
{
   if (exp.size() == 0 || exp.size() != sim.size())
      return std::numeric_limits<double>::max();

   double chi2 = 0;
   for (int i = 0; i < exp.size(); ++i) {
      double chi = (exp.at(i) - par[0] * sim.at(i)) / (0.1 * exp.at(i));
      chi2 += chi * chi;
   }

   chi2 /= exp.size();
   LOG(debug) << "A: " << par[0] << " Chi2: " << chi2;
   return chi2;
}

double
AtMCFission::ObjectiveChargeDiff2(const std::vector<double> &exp, const std::vector<double> &sim, const double *par)
{
   if (exp.size() == 0)
      return std::numeric_limits<double>::max();

   double chi2 = 0;
   double Qtot = 0;
   for (int i = 0; i < exp.size(); ++i) {
      chi2 += (exp[i] - par[0] * sim[i]) * (exp[i] - par[0] * sim[i]);
      Qtot += exp[i];
   }

   chi2 /= Qtot;
   LOG(debug) << "A: " << par[0] << " Chi2: " << chi2;
   return chi2;
}

double AtMCFission::ObjectiveCharge(const std::array<std::vector<double>, 2> &expFull,
                                    const std::array<std::vector<double>, 2> &simFull, AtMCResult &definition)
{
   // Start by trimming the two FFs down by removing the zeros in the charge and
   // combining into a single array. Don't examine things within 6 TB of the pad plane
   std::vector<double> exp;
   std::vector<double> sim;
   for (int i = 0; i < 2; ++i) {
      for (int tb = 80; tb < 512; ++tb) {
         LOG(debug) << i << "," << tb << " " << expFull[i][tb] << " " << simFull[i][tb];
         if (expFull[i][tb] != 0) {
            exp.push_back(expFull[i][tb]);
            sim.push_back(simFull[i][tb]);
         }
      }
   }

   if (fFitAmp && !(exp.size() == 0 || exp.size() != sim.size())) {
      LOG(info) << exp.size() << " " << sim.size();
      auto functor = ROOT::Math::Functor(std::bind(fObjCharge, exp, sim, std::placeholders::_1), 1); // NOLINT

      std::vector<double> A = {fAmp};
      ROOT::Fit::Fitter fitter;
      fitter.Config().SetMinimizer("Minuit2");
      fitter.SetFCN(functor, A.data());
      bool ok = fitter.FitFCN();
      if (!ok) {
         LOG(error) << "Failed to fit the charge curves. Not adding to objective.";
         return std::numeric_limits<double>::max();
      }
      auto &result = fitter.Result();
      double amp = result.Parameter(0);
      definition.fParameters["Amp"] = amp;
      definition.fParameters["qChi2"] = result.MinFcnValue();
      return result.MinFcnValue();
   } else {

      definition.fParameters["Amp"] = fAmp;
      std::vector<double> A = {fAmp};
      auto chi2 = fObjCharge(exp, sim, A.data());
      definition.fParameters["qChi2"] = chi2;
      return chi2;
   }
}

double AtMCFission::ObjectivePosition(const AtFissionEvent &expEvent, int SimEventID)
{
   AtEvent &simEvent = fEventArray.at(SimEventID);

   // Sort the hits by pad number
   simEvent.SortHitArray();
   auto fragHits = expEvent.GetFragHits();
   LOG(debug) << fragHits.size() << " " << expEvent.GetFragHits(0).size() << " " << expEvent.GetFragHits(1).size();

   std::sort(fragHits.begin(), fragHits.end(),
             [](const AtHit *a, const AtHit *b) { return a->GetPadNum() < b->GetPadNum(); });

   auto simHit = simEvent.GetHits().begin();

   auto map = fPulse->GetMap();
   double chi2 = 0;
   int nPads = 0;

   // Get every pad that was in the experimental event (assumes that each pad was only used once)
   for (auto expHit : fragHits) {
      auto padNum = expHit->GetPadNum();

      // Only keep hits where the pad is not inhibited in some way
      if (fMap->IsInhibited(padNum) != AtMap::InhibitType::kNone)
         continue;

      // Look for pad in simulated event
      auto hitIt = std::find_if(simHit, simEvent.GetHits().end(),
                                [padNum](const AtEvent::HitPtr &hit2) { return hit2->GetPadNum() == padNum; });
      if (hitIt == simEvent.GetHits().end() || expHit->GetPositionSigma().Z() == 0) {
         chi2++; // This is the integral of the normalized gaussian that we have nothing to compare to
         nPads++;
         LOG(debug) << "Did not find pad " << padNum << " in the simulated event or it's in the beam region";
         continue;
      }

      // Update the chi2 with this hit

      simHit = hitIt;
      double simZ = simHit->get()->GetPosition().Z();
      double simSig = simHit->get()->GetPositionSigma().Z();
      double expZ = expHit->GetPosition().Z();
      double expSig = expHit->GetPositionSigma().Z();

      double diff = ObjectivePosition(expZ, expSig, simZ, simSig);
      LOG(debug) << "Found " << padNum << " in the simulated event. Simulated hit is " << simZ << " +- " << simSig
                 << "  Experimental hit is " << expZ << " +- " << expSig << " Chi2 between pads is " << diff;
      chi2 += diff;
      nPads++;
   }
   return chi2 / nPads;
}
/**
 * Returns \Integral_{-inf}^{inf} (G[uO,sO,x] - G[uE,sE,x])^2/G[uE,sE,x] dx
 * Assuming the Gaussain functions are normalized (same area).
 *
 * The experimental is the expected, the observed is our simulated event
 */
double AtMCFission::ObjectivePosition2(double uE, double sE, double uO, double sO)
{
   double num = sE * sE * std::exp(((uE - uO) * (uE - uO)) / (2 * sE * sE - sO * sO));
   double denom = sO * std::sqrt(2 * sE * sE - sO * sO);
   return num / denom - 1;
}

/**
 * Returns \Integral_{-inf}^{inf} (G[uO,sO,x] - G[uE,sE,x])^2 dx
 * Assuming the Gaussain functions have the same height.
 *
 * The experimental is the expected, the observed is our simulated event
 */
double AtMCFission::ObjectivePosition4(double uE, double sE, double uO, double sO)
{
   double num = 2 * sqrt(2) * sE * sO * std::exp((-(uE - uO) * (uE - uO)) / (2 * (sE * sE + sO * sO)));
   double denom = std::sqrt(sE * sE + sO * sO);
   return sE + sO - num / denom;
}

/**
 * Returns \Integral_{-inf}^{inf} (G[uO,sO,x] - G[uE,sE,x])^2 dx
 * Assuming the Gaussain functions have the same area.
 *
 * The experimental is the expected, the observed is our simulated event
 */
double AtMCFission::ObjectivePosition(double uE, double sE, double uO, double sO)
{
   double num = 2 * sqrt(2) * std::exp((-(uE - uO) * (uE - uO)) / (2 * (sE * sE + sO * sO)));
   double denom = std::sqrt(sE * sE + sO * sO);
   return 1 / sE + 1 / sO - num / denom;
}

/**
 * Returns \Integral_{-inf}^{inf} (G[uO,sO,x] - G[uE,sE,x])^2/G[uE,sE,x] dx
 * Assuming the Gaussain functions have the same height.
 *
 * The experimental is the expected, the observed is our simulated event
 */
double AtMCFission::ObjectivePosition3(double uE, double sE, double uO, double sO)
{
   double num = sE * sO * std::exp(((uE - uO) * (uE - uO)) / (2 * sE * sE - sO * sO));
   double denom = std::sqrt(2 * sE * sE - sO * sO);
   return sE - 2 * sO + num / denom;
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
 * Gets the beam direction assuming the velocity of the FFs are the same (i.e. it assumes the beam
 * direction is half way between the two FF in tha lab frame.
 */
XYZVector AtMCFission::GetBeamDirSameV(AtMCResult &res, const std::array<XYZVector, 2> &ffDir)
{
   auto beamInLabFrame = (ffDir[0].Unit() + ffDir[1].Unit()) / 2;
   res.fParameters["beamX"] = beamInLabFrame.Unit().X();
   res.fParameters["beamY"] = beamInLabFrame.Unit().Y();
   res.fParameters["beamZ"] = beamInLabFrame.Unit().Z();
   res.fParameters["beamSel"] = 1;
   return beamInLabFrame;
}

/**
 * Gets the beam direction using kinematics constraints given our assumtion on the kinetic energy of
 * the fission fragments from viola systematics, the masses of the fission fragments, and the folding
 * angle between the fission fragments.
 */
XYZVector AtMCFission::GetBeamDir(AtMCResult &res, const std::array<XYZVector, 2> &ffDir, double pTrans)
{
   using namespace AtTools::Kinematics;

   // Get the transverse velocity of the fission fragments
   double v1 = GetBeta(pTrans, AtoE(res.fParameters["A0"]));
   double v2 = GetBeta(pTrans, AtoE(res.fParameters["A1"]));

   // Get the folding angle between the fision fragments
   auto foldingAngle = ROOT::Math::VectorUtil::Angle(ffDir[0], ffDir[1]);

   // The velocity of the beam is related to the folding angle and the transverse velocities
   //  Here we assume that we are at non-relativistic velocities, or the decay angle is closs to
   //  90 degrees (basically (beta of the beam * beta of FF in z direction in CoM frame) is zero)
   auto tanA = std::tan(foldingAngle);
   auto s = std::sqrt(v1 * v1 + 4 * v1 * v2 * tanA * tanA + 2 * v1 * v2 + v2 * v2);
   auto vBeam = (v1 + v2 + s) / (2 * tanA);

   // The angle between FF1 and the beam is given by
   auto angle1 = std::atan(v1 / vBeam);
   auto angle2 = std::atan(v2 / vBeam);
   LOG(debug) << "Angle 1: " << angle1 * TMath::RadToDeg() << " Angle 2: " << angle2 * TMath::RadToDeg();
   LOG(debug) << "Folding angle: " << foldingAngle * TMath::RadToDeg() << " Beam v: " << vBeam;

   // The beam direction should be the FF direction vector rotated around the normal vector to the plane
   // containing the fission fragments by the angle between the FF direction and the beam.

   auto norm = ffDir[0].Cross(ffDir[1]).Unit();
   auto rot1 = ROOT::Math::AxisAngle(norm, angle1);
   auto beamDir = rot1(ffDir[0]);
   res.fParameters["beamX"] = beamDir.Unit().X();
   res.fParameters["beamY"] = beamDir.Unit().Y();
   res.fParameters["beamZ"] = beamDir.Unit().Z();
   res.fParameters["beamSel"] = 0;

   LOG(debug) << "Beam dir: " << beamDir;

   LOG(debug) << "Angle 1: " << ROOT::Math::VectorUtil::Angle(ffDir[0], beamDir) * TMath::RadToDeg()
              << " Angle 2: " << ROOT::Math::VectorUtil::Angle(ffDir[1], beamDir) * TMath::RadToDeg();
   return beamDir;
}

XYZVector AtMCFission::GetBeamDirSample(AtMCResult &res, const std::array<XYZVector, 2> &ffDir)
{
   // Sample the deviation of the beam away from the nominal beam direction
   Polar3D beamDir(1, res.fParameters["thBeam"], 0);
   LOG(info) << "Sampled beam direction (deviation from nominal)"
             << ROOT::Math::VectorUtil::Angle(XYZVector(0, 0, 1), beamDir) * TMath::RadToDeg() << " "
             << beamDir.Theta() * TMath::RadToDeg();

   // Get our nominal beam direction projected into the plane of the reaction
   auto norm = ffDir[0].Cross(ffDir[1]).Unit();
   auto projBeam = fNominalBeamDir - fNominalBeamDir.Dot(norm) * norm;

   // Rotate the beam vector so our deviation is from the nominal beam axis
   auto rot = AtTools::GetRotationToZ(projBeam);
   auto beamInLabFrame = XYZVector(rot.Inverse()(beamDir));
   // auto beamInLabFrame = projBeam;
   LOG(info) << " Sampled beam deviates from nominal by "
             << ROOT::Math::VectorUtil::Angle(beamInLabFrame, projBeam) * TMath::RadToDeg() << " deg.";

   // Set the beam direction in the result file as well
   res.fParameters["beamX"] = beamInLabFrame.Unit().X();
   res.fParameters["beamY"] = beamInLabFrame.Unit().Y();
   res.fParameters["beamZ"] = beamInLabFrame.Unit().Z();
   res.fParameters["beamSel"] = 2;
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
   // This will sample all the defined patameter distributions and save the results in
   // the returned AtMCResult
   AtMCResult result = AtMCFitter::DefineEvent();

   // Translate the sampled Z into the particle ID of the FF.
   int Z1 = std::round(result.fParameters["Z"]);
   while (Z1 > fZmax || Z1 < fZmin) {
      result.fParameters["Z"] = fParameters["Z"]->Sample();
      Z1 = std::round(result.fParameters["Z"]);
   }
   int A1 = std::round(fCN.A * (double)Z1 / fCN.Z);
   result.fParameters["Z0"] = Z1;
   result.fParameters["A0"] = A1;
   result.fParameters["Z1"] = fCN.Z - Z1;
   result.fParameters["A1"] = fCN.A - A1;

   result.fParameters["thCM"] = 90 * TMath::DegToRad(); // Decay angle in CoM frame
   return result;
}

/**
 * Set the magnitude of the FF momenta in the lab' frame (assume the beam direction points
 * in the Z direction).
 * The transverse momentum (w.r.t. the beam axis) of the fission fragment in the lab frame
 * is the same as in the CoM frame.
 * We can calculate the magnitude of p for a FF by comparing the angle between the beam and FF.
 */
void AtMCFission::SetMomMagnitude(std::array<XYZVector, 2> &moms, double pTrans)
{
   for (auto &mom : moms) {
      auto angle = mom.Theta();
      LOG(debug) << "Angle: " << mom.Theta() * TMath::RadToDeg();
      auto p = pTrans / std::sin(angle);
      mom = mom.Unit() * p;
   }
}

TClonesArray AtMCFission::SimulateEvent(AtMCResult &def)
{
   using namespace AtTools::Kinematics;
   fSim->NewEvent();

   // Set the magnitude of the space charge for this event.
   // TODO: Make this thread safe (deep copy simplesim, and once per thread)
   auto radialModel = dynamic_cast<AtRadialChargeModel *>(fSim->GetSpaceChargeModel().get());
   auto lineModel = dynamic_cast<AtLineChargeModel *>(fSim->GetSpaceChargeModel().get());
   if (radialModel) {
      radialModel->SetDistortionField(AtLineChargeZDep(def.fParameters["lambda"]));
      LOG(debug) << "Setting Lambda: " << def.fParameters["lambda"];

   } else if (lineModel) {
      lineModel->SetLambda(def.fParameters["lambda"]);
      LOG(debug) << "Setting Lambda: " << def.fParameters["lambda"];

   } else
      LOG(debug) << "No space charge to apply.";

   auto vertex = GetVertex(def);
   LOG(debug) << "Setting vertex: " << vertex;

   auto fragID = GetFragmentSpecies(def, fCN);
   LOG(debug) << "Setting fragment 1: " << fragID[0].Z << " " << fragID[0].A;
   LOG(debug) << "Setting fragment 2: " << fragID[1].Z << " " << fragID[1].A;

   // We have the masses, now calculate the momentum of these two fragments in the CoM frame. We will
   // fudge and say the mass of a fragment is A amu (neglecting binding energy).
   double KE = violaEn(fCN.A, fCN.Z);
   double gamma1 = GetGamma(KE, AtoE(fragID[0].A), AtoE(fragID[1].A));
   double p1 = GetRelMom(gamma1, AtoE(fragID[0].A));
   p1 *= sin(def.fParameters["thCM"]);

   LOG(debug) << "v1: " << GetBeta(gamma1) << " v2: " << GetBeta(GetGamma(KE, AtoE(fragID[1].A), AtoE(fragID[0].A)));

   // Get the momentum direction for the FF and beam in the lab frame
   auto mom = GetMomDirLab(def); // Pulled from the data
                                 // auto beamDir = GetBeamDirSameV(def, mom);
   auto beamDir = GetBeamDir(def, mom, p1);

   LOG(debug) << "p1: " << mom[0];
   LOG(debug) << "p2: " << mom[1];
   LOG(debug) << "b:  " << beamDir;
   LOG(debug) << "p1 * b: " << mom[0].Dot(beamDir);
   LOG(debug) << "p2 * b: " << mom[1].Dot(beamDir);
   LOG(debug) << "Transforming to Lab'";

   // Transform the momentum into the Lab' frame (beamDir points along z)
   auto toLabPrime = AtTools::GetRotationToZ(beamDir);
   beamDir = toLabPrime(beamDir);
   for (int i = 0; i < 2; ++i)
      mom[i] = toLabPrime(mom[i]);

   LOG(debug) << "p1: " << mom[0];
   LOG(debug) << "p2: " << mom[1];
   LOG(debug) << "b:  " << beamDir;
   LOG(debug) << "p1 * b: " << mom[0].Dot(beamDir);
   LOG(debug) << "p2 * b: " << mom[1].Dot(beamDir);

   // Set the momentum of the fission fragments in the lab' frame
   SetMomMagnitude(mom, p1);
   std::vector<XYZEVector> ffMom(2);
   ffMom[0] = Get4Vector(mom[0], AtoE(fragID[0].A));
   ffMom[1] = Get4Vector(mom[1], AtoE(fragID[1].A));

   auto pBeam = ffMom[0] + ffMom[1];

   LOG(debug) << "Frag A: " << ffMom[0] << " mass " << EtoA(ffMom[0].M()) << " energy: " << ffMom[0].E() - ffMom[0].M()
              << " beta: " << ffMom[0].Beta();
   LOG(debug) << "Frag B: " << ffMom[1] << " mass " << EtoA(ffMom[1].M()) << " energy: " << ffMom[1].E() - ffMom[1].M()
              << " beta: " << ffMom[1].Beta();
   LOG(debug) << "Beam: " << pBeam << " mass " << EtoA(pBeam.M()) << " energy: " << pBeam.E() - pBeam.M();

   LOG(debug) << "Transforming back to lab frame";

   // Transform the momentum of the FF back into the lab frame.
   for (int i = 0; i < 2; ++i)
      ffMom[i] = toLabPrime.Inverse()(ffMom[i]);

   // Use conservation of 4 momentum to get the beam momentum
   pBeam = ffMom[0] + ffMom[1];

   LOG(debug) << "Frag A: " << ffMom[0] << " mass " << EtoA(ffMom[0].M()) << " energy: " << ffMom[0].E() - ffMom[0].M()
              << " beta: " << ffMom[0].Beta();
   LOG(debug) << "Frag B: " << ffMom[1] << " mass " << EtoA(ffMom[1].M()) << " energy: " << ffMom[1].E() - ffMom[1].M()
              << " beta: " << ffMom[1].Beta();
   LOG(debug) << "Beam: " << pBeam << " mass " << EtoA(pBeam.M()) << " energy: " << pBeam.E() - pBeam.M();
   def.fParameters["EBeam"] = pBeam.E() - pBeam.M();

   for (int i = 0; i < 2; ++i)
      fSim->SimulateParticle(fragID[i].Z, fragID[i].A, vertex, ffMom[i]);

   return fSim->GetPointsArray();
}
