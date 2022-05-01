#include "AtGaussian.h"

#include "AtHit.h"
#include "AtSample.h" // for RandomSample

#include <Math/PdfFuncMathCore.h>
#include <Math/Point3D.h>  // for operator-
#include <Math/Vector3D.h> // for DisplacementVector3D

#include <cmath> // for sqrt

using namespace RandomSample;
std::vector<double> AtGaussian::PDF(const AtHit &hit)
{
   auto dist = (fReferenceHit.GetPosition() - hit.GetPosition()).Mag2();
   dist = std::sqrt(dist);
   return {ROOT::Math::gaussian_pdf(dist, fSigma)};
}
