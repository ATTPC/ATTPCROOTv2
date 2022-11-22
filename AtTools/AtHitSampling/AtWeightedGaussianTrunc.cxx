#include "AtWeightedGaussianTrunc.h"

#include "AtHit.h"
#include "AtSample.h" // for RandomSample

#include <Math/Point3D.h>  // for operator-
#include <Math/Vector3D.h> // for DisplacementVector3D
#include <TRandom.h>

#include <algorithm>
#include <cmath> // for sqrt
using namespace RandomSample;

std::vector<AtHit> AtWeightedGaussianTrunc::SampleHits(int N)
{
   int p1, p2;
   int pclouds = fHits->size();
   int counter = 0;
   double dist = 0;
   double sigma = 30.0;
   double y = 0;
   double gauss = 0;
   double w = 0;
   double Tcharge = 0;
   double avgCharge = 0;
   std::vector<double> Proba;
   std::vector<AtHit> retVec;

   for (int i = 0; i < pclouds; i++)
      Tcharge += fHits->at(i)->GetCharge();

   if (Tcharge > 0)
      for (int i = 0; i < pclouds; i++)
         Proba.push_back(fHits->at(i)->GetCharge());

   avgCharge = Tcharge / (double)pclouds;
   p1 = gRandom->Uniform() * pclouds;
   retVec.push_back(*fHits->at(p1));

   do {
      counter++;
      p2 = gRandom->Uniform() * pclouds;
      if (p2 == p1)
         continue;
      dist = std::sqrt((fHits->at(p1)->GetPosition() - fHits->at(p2)->GetPosition()).Mag2());
      gauss = 1.0 * exp(-1.0 * pow(dist / sigma, 2));
      y = gRandom->Uniform();
      w = gRandom->Uniform() * 4. * avgCharge;
      if (fHits->at(p2)->GetCharge() > w || y < gauss) {
         retVec.push_back(*fHits->at(p2));
      }
   } while (retVec.size() < N && counter < pclouds && counter < 50);

   return retVec;
}

std::vector<double> AtWeightedGaussianTrunc::PDF(const AtHit &hit)
{
   return {};
}
