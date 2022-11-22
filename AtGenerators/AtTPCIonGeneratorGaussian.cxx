#include "AtTPCIonGeneratorGaussian.h"

#include <TRandom.h>

#include <algorithm> // for clamp
#include <cmath>     // for cos, sin, asin

ClassImp(AtTPCIonGeneratorGaussian);

void AtTPCIonGeneratorGaussian::SetBeamLimits(Double32_t r, Double32_t z, Double_t theta)
{
   fR = r;
   fz = z;
   fTheta = theta;
}

void AtTPCIonGeneratorGaussian::SetVertexCoordinates()
{
   double pi = 2 * asin(1.0);

   Double_t radius = std::clamp(gRandom->Gaus(0, fR / 3), 0.0, fR);
   Double_t phi_R = gRandom->Uniform(0, 2 * pi);
   fVx = radius * cos(phi_R);
   fVy = radius * sin(phi_R);

   Double_t theta = gRandom->Uniform(0, fTheta);
   Double_t pr = fPz * sin(theta);
   fPz *= cos(theta);
   fPx = pr * cos(phi_R);
   fPy = pr * sin(phi_R);
}
