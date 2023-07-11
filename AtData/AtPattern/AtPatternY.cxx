#include "AtPatternY.h"
// IWYU pragma: no_include <ext/alloc_traits.h>
#include "AtPatternLine.h" // for AtPatternLine::XYZPoint, AtPatter...

#include <FairLogger.h>

#include <Math/Functor.h>
#include <Math/Point3D.h>  // for DisplacementVector3D, operator*
#include <Math/Vector3D.h> // for DisplacementVector3D, operator*
#include <TEveCompound.h>
#include <TEveElement.h>
#include <TEveLine.h>
#include <TEvePointSet.h>
#include <TString.h> // for Form

#include <Fit/FitConfig.h> // for FitConfig
#include <Fit/FitResult.h> // for FitResult
#include <Fit/Fitter.h>
#include <Fit/ParameterSettings.h> // for ParameterSettings
#include <algorithm>               // for max, min_element, sort
#include <cmath>                   // for cos, sin, pow, sqrt, acos, atan, fabs
#include <set>                     // for set, _Rb_tree_const_iterator
#include <utility>                 // for swap

using namespace AtPatterns;

ClassImp(AtPatternY);

AtPatternY::AtPatternY() : AtPattern(4) {}

TEveElement *AtPatternY::GetEveElement() const
{
   auto shape = new TEveCompound(); // NOLINT

   // Any element added with this color will be changed when the color of shape is changed later
   shape->SetMainColor(kGreen);
   shape->CSCApplyMainColorToMatchingChildren();
   shape->OpenCompound();

   // Add lines to shape
   auto beam = fBeam.GetEveElement();
   dynamic_cast<TEveLine *>(beam)->SetName("beam");
   beam->SetMainColor(kRed);
   shape->AddElement(beam);

   for (int i = 0; i < 2; ++i) {
      auto elem = fFragments[i].GetEveElement();
      dynamic_cast<TEveLine *>(elem)->SetName(Form("frag_%d", i));
      elem->SetMainColor(kGreen);
      shape->AddElement(elem);
   }

   // Add vertex to shape
   auto vertex = new TEvePointSet("vertex", 1); // NOLINT
   vertex->SetOwnIds(true);
   vertex->SetMarkerSize(2);
   vertex->SetMainColor(kGreen);
   vertex->SetNextPoint(fBeam.GetPoint().X() / 10, fBeam.GetPoint().Y() / 10, fBeam.GetPoint().Z() / 10);
   shape->AddElement(vertex);

   shape->CloseCompound();
   return shape;
}

AtPatternY::XYZPoint AtPatternY::ClosestPointOnPattern(const XYZPoint &point) const
{
   // create a set of pairs with points and distance to pattern sorted by distance
   auto comp = [](const std::pair<XYZPoint, double> &a, const std::pair<XYZPoint, double> &b) {
      return a.second < b.second;
   };
   auto points = std::set<std::pair<XYZPoint, double>, decltype(comp)>(comp);

   points.insert({fBeam.ClosestPointOnPattern(point), fBeam.DistanceToPattern(point)});
   for (const auto &ray : fFragments)
      points.insert({ray.ClosestPointOnPattern(point), ray.DistanceToPattern(point)});

   // Get the closest point to the pattern
   return points.begin()->first;
}

Double_t AtPatternY::DistanceToPattern(const XYZPoint &point) const
{
   std::vector<double> distances;
   distances.push_back(fBeam.DistanceToPattern(point));
   for (const auto &ray : fFragments)
      distances.push_back(ray.DistanceToPattern(point));
   return *std::min_element(distances.begin(), distances.end());
}

int AtPatternY::GetPointAssignment(const XYZPoint &point) const
{
   auto comp = [](const std::pair<int, double> &a, const std::pair<int, double> &b) { return a.second < b.second; };
   auto points = std::set<std::pair<int, double>, decltype(comp)>(comp);

   for (int i = 0; i < fFragments.size(); ++i)
      points.insert({i, fFragments[i].DistanceToPattern(point)});
   points.insert({fFragments.size(), fBeam.DistanceToPattern(point)});

   return points.begin()->first;
}

std::vector<double> AtPatternY::GetPatternPar() const
{
   std::vector<double> ret;
   ret.resize(12);
   ret[0] = GetVertex().X();
   ret[1] = GetVertex().Y();
   ret[2] = GetVertex().Z();
   ret[3] = GetBeamDirection().X();
   ret[4] = GetBeamDirection().Y();
   ret[5] = GetBeamDirection().Z();
   for (int i = 0; i < 2; ++i) {
      ret[6 + 3 * i] = GetFragmentDirection(i).X();
      ret[7 + 3 * i] = GetFragmentDirection(i).Y();
      ret[8 + 3 * i] = GetFragmentDirection(i).Z();
   }
   return ret;
}

/**
 * Defines the pattern using the following parameters
 * vertex = (par[0], par[1], par[2])
 * beamDir = (par[3], par[4], par[5])
 * fragDir[0] = (par[6], par[7], par[8])
 * fragDir[1] = (par[9], par[10], par[11])
 */
void AtPatternY::DefinePattern(std::vector<double> par)
{
   if (par.size() != 12) {
      LOG(error) << "Defining a Y pattern requires 12 parameters!";
      return;
   }

   XYZPoint vertex(par[0], par[1], par[2]);
   XYZVector beamDir(par[3], par[4], par[5]);
   std::array<XYZVector, 2> fragDir;
   for (int i = 0; i < 2; ++i)
      fragDir[i] = {par[6 + i * 3], par[7 + i * 3], par[8 + i * 3]};
   DefinePattern(vertex, beamDir, fragDir);
}

void AtPatternY::DefinePattern(const XYZPoint &vertex, const XYZVector &beamDir,
                               const std::array<XYZVector, 2> &fragDir)
{
   fBeam.DefinePattern(vertex, beamDir);
   for (int i = 0; i < 2; ++i)
      fFragments[i].DefinePattern(vertex, fragDir[i]);
}

void AtPatternY::DefinePattern(const std::vector<XYZPoint> &points)
{
   if (points.size() != fNumPoints)
      LOG(fatal) << "Trying to create model with wrong number of points " << points.size();

   // Sort points by z location in decreasing order
   auto sortedPoints = points;
   std::sort(sortedPoints.begin(), sortedPoints.end(),
             [](const ROOT::Math::XYZPoint &a, const ROOT::Math::XYZPoint &b) { return a.Z() > b.Z(); });

   // Vertex point is defined to be the one at second largest z
   auto fVertex = points[1];
   // Get the vector pointing from the vertex to the next other beam point
   auto dir = points[0] - fVertex;
   if (dir.Z() < 0)
      dir.SetZ(-dir.Z());
   fBeam.DefinePattern(fVertex, dir);

   for (int i = 0; i < 2; ++i) {
      fFragments[i].DefinePattern({fVertex, points[i + 2]});
   }
}

/**
 * @brief Get point along the beam axis at z
 *
 * The parameter passed has no defined physical interpretation.
 *
 * @param[in] z Location of point at parameter.
 */
AtPatternY::XYZPoint AtPatternY::GetPointAt(double z) const
{
   return dynamic_cast<const AtPatternLine *>(&fBeam)->GetPointAt(z);
}

// charge may be empty, if so do not do a charge weighted fit.
void AtPatternY::FitPattern(const std::vector<XYZPoint> &points, const std::vector<double> &charge)
{
   // Based on the example from ROOT documentation https://root.cern.ch/doc/master/line3Dfit_8C_source.html

   bool weighted = points.size() == charge.size();
   LOG(debug) << "Fitting with" << (weighted ? " " : "out ") << "charge weighting";

   // This functor is what we are minimizing. It takes in the model parameters and defines an example
   // of this pattern based on the model parameters. It then loops through every hit associated with
   // pattern and calculates the chi2.
   auto func = [&points, &charge, weighted](const double *par) {
      AtPatternY pat;
      pat.DefinePattern(std::vector<double>(par, par + 12));
      double chi2 = 0;
      double qTot = 0;
      for (int i = 0; i < points.size(); ++i) {
         auto q = weighted ? charge[i] : 1;
         chi2 += pat.DistanceToPattern(points[i]) * pat.DistanceToPattern(points[i]) * q;
         qTot += q;
      }

      return fabs(chi2 / qTot);
   };

   auto functor = ROOT::Math::Functor(func, 12);

   ROOT::Fit::Fitter fitter;
   auto iniPar = GetPatternPar();

   LOG(debug) << "Initial parameters";
   for (int i = 0; i < iniPar.size(); ++i)
      LOG(debug) << Form("Par_%d", i) << "\t = " << iniPar[i];

   fitter.SetFCN(functor, iniPar.data());

   // Constrain the Z direction to be the same
   for (int i = 0; i < 3; ++i)
      fitter.Config().ParSettings(5 + i * 3).Fix();

   for (int i = 0; i < 12; ++i)
      fitter.Config().ParSettings(i).SetStepSize(.01);

   bool ok = fitter.FitFCN();
   if (!ok) {
      LOG(error) << "Failed to fit the pattern, using result of SAC";
      DefinePattern(iniPar);
      return;
   }

   auto &result = fitter.Result();
   DefinePattern(result.Parameters());
   fChi2 = result.MinFcnValue();
   fNFree = points.size() - result.NPar();
}
