#include "AtWeightedGaussian.h"

#include "AtHit.h"
#include "AtSample.h" // for RandomSample

#include <Math/PdfFuncMathCore.h>
#include <Math/Point3D.h>  // for operator-
#include <Math/Vector3D.h> // for DisplacementVector3D

#include <cmath>   // for sqrt
#include <utility> // for move

using namespace RandomSample;
void AtWeightedGaussian::SetHitsToSample(const std::vector<const AtHit *> &hits)
{
   AtSampleFromReference::SetHitsToSample(hits);
   fChargeSample.SetHitsToSample(hits);
}

std::vector<double> AtWeightedGaussian::PDF(const AtHit &hit)
{
   auto dist = (fReferenceHit.GetPosition() - hit.GetPosition()).Mag2();
   dist = std::sqrt(dist);
   return {ROOT::Math::gaussian_pdf(dist, fSigma), hit.GetCharge()};
}

void AtWeightedGaussian::SampleReferenceHit()
{
   AtChargeWeighted charge;
   charge.SetHitsToSample(*fHits);
   SetReferenceHit(std::move(charge.SampleHits(1)[0]));
}
