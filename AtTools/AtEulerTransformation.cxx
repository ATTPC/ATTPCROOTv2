#include "AtEulerTransformation.h"

#include <cmath>
#include <iostream>

#include <Rtypes.h>

void AtEulerTransformation::DoTheEulerTransformationBeam2Lab()
{

   // ***  Transformation  coordinate system (Euler angles) :  ***
   // DEFINITIONS:
   //  --from beam to lab system
   //     alpha = phi
   //     beta  = theta
   //     gamma = 0
   // *** inputs: theta_beam, phi_beam, alpha, beta, gamma
   // *** calculate: theta_lab, phi_lab (from beam to lab)
   //
   //  --or from lab to beam system
   //     alpha = pi
   //     beta  = theta_beam
   //     gamma = pi - phi_beam

   // BE CAREFUL of the units! Here the angles are not in degrees!

   const Double_t PI = 3.14159265358979323846;
   //         const G4double rad=180.0/PI;

   Double_t Theta0 = ThetaInBeamSystem;
   Double_t Phi0 = PhiInBeamSystem;

   Double_t alpha = BeamDirectionAtVertexPhi;  // alpha=phi
   Double_t beta = BeamDirectionAtVertexTheta; // beta =theta
   Double_t gamma = 0.0;

   Double_t px = sin(Theta0) * cos(Phi0);
   Double_t py = sin(Theta0) * sin(Phi0);
   Double_t pz = cos(Theta0);

   Double_t sa = sin(alpha);
   Double_t ca = cos(alpha);
   Double_t sb = sin(beta);
   Double_t cb = cos(beta);
   Double_t sg = sin(gamma);
   Double_t cg = cos(gamma);

   Double_t px_1 = px * (ca * cb * cg - sa * sg) + py * (-ca * cb * sg - sa * cg) + pz * (ca * sb);
   Double_t py_1 = px * (sa * cb * cg + ca * sg) + py * (-sa * cb * sg + ca * cg) + pz * (sa * sb);
   Double_t pz_1 = px * (-sb * cg) + py * (sb * sg) + pz * (cb);

   Double_t pxy_1 = sqrt(px_1 * px_1 + py_1 * py_1);
   Double_t sithe1 = pxy_1;
   Double_t cothe1 = pz_1;

   Double_t theta_1 = atan2(sithe1, cothe1);

   ThetaInLabSystem = theta_1;

   Double_t phi_1 = 0.0;

   if (sithe1 != 0.0) {
      Double_t siphi1 = py_1 / pxy_1;
      Double_t cophi1 = px_1 / pxy_1;
      phi_1 = atan2(siphi1, cophi1);

      if (phi_1 < 0.0) {
         phi_1 = 2. * PI + phi_1;
      }
   }

   if (sithe1 == 0.0) {
      phi_1 = 0.0;
   }

   PhiInLabSystem = phi_1;
}

void AtEulerTransformation::Dump() {}

void AtEulerTransformation::PrintResults()
{
   std::cout << "Beam direction in the Lab frame:" << std::endl
             << "  theta=" << BeamDirectionAtVertexTheta << ", phi=" << BeamDirectionAtVertexPhi << std::endl;
   std::cout << "Particle direction in the beam system:" << std::endl
             << "  theta=" << ThetaInBeamSystem << ", phi=" << PhiInBeamSystem << std::endl;
   std::cout << "Particle direction in the Lab system:" << std::endl
             << "  theta=" << ThetaInLabSystem << ", phi=" << PhiInLabSystem << std::endl;
}

ClassImp(AtEulerTransformation)
