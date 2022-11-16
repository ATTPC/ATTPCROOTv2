#include "AtKinematics.h"

#include <Rtypes.h> // for TGenericClassInfo
#include <TMath.h>  // for Power, Sqrt

#include <cmath>    // for pow, sqrt, cos
#include <iostream> // for operator<<, basic_ostream, basic_ostream<>::__os...

ClassImp(AtTools::AtKinematics);

AtTools::AtKinematics::AtKinematics() : fVerbosity(0)
{

   std::cout << " AtTools::AtKinematics: Initializing Kinematical Fit matrices "
             << "\n";

   auto alpha0 = std::make_unique<TMatrixD>(4 * fNumParticles, 1);
   fAlphaP.push_back(std::move(alpha0));

   for (auto i = 0; i < fNumParticles; ++i) {
      auto alpha = std::make_unique<TMatrixD>(4, 1);
      fAlphaP.push_back(std::move(alpha));
   }
}

std::tuple<Double_t, Double_t> AtTools::AtKinematics::GetMomFromBrho(Double_t M, Double_t Z, Double_t brho)
{

   const Double_t M_Ener = M * 931.49401 / 1000.0;
   Double_t p = brho * Z * (2.99792458 / 10.0); // In GeV
   Double_t E = TMath::Sqrt(TMath::Power(p, 2) + TMath::Power(M_Ener, 2)) - M_Ener;
   if (fVerbosity == 1)
      std::cout << " Brho : " << brho << " - p : " << p << " - E : " << E << "\n";
   return std::make_tuple(p, E);
}

Double_t AtTools::AtKinematics::TwoBodyEx(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj,
                                          Double_t thetalab, Double_t K_eject)
{
   // in this definition: m1(projectile); m2(target); m3(ejectile); and m4(recoil);
   double Et1 = K_proj + m1;
   double Et2 = m2;
   double Et3 = K_eject + m3;
   // double Et4 = Et1 + Et2 - Et3;
   double m4_ex, Ex, theta_cm;
   double s, t, u; //---Mandelstam variables

   s = pow(m1, 2) + pow(m2, 2) + 2 * m2 * Et1;
   u = pow(m2, 2) + pow(m3, 2) - 2 * m2 * Et3;

   m4_ex = sqrt((cos(thetalab) * omega(s, pow(m1, 2), pow(m2, 2)) * omega(u, pow(m2, 2), pow(m3, 2)) -
                 (s - pow(m1, 2) - pow(m2, 2)) * (pow(m2, 2) + pow(m3, 2) - u)) /
                   (2 * pow(m2, 2)) +
                s + u - pow(m2, 2));
   Ex = m4_ex - m4;

   // t = pow(m2, 2) + pow(m4_ex, 2) - 2 * m2 * Et4;

   // for inverse kinematics Note: this angle corresponds to the recoil
   /*theta_cm = TMath::Pi() - acos((pow(s, 2) + s * (2 * t - pow(m1, 2) - pow(m2, 2) - pow(m3, 2) - pow(m4_ex, 2)) +
                                  (pow(m1, 2) - pow(m2, 2)) * (pow(m3, 2) - pow(m4_ex, 2))) /
                                 (omega(s, pow(m1, 2), pow(m2, 2)) * omega(s, pow(m3, 2), pow(m4_ex, 2))));


    THcm = theta_cm*TMath::RadToDeg();
   */
   return Ex;
}

Double_t AtTools::AtKinematics::omega(Double_t x, Double_t y, Double_t z)
{
   return sqrt(x * x + y * y + z * z - 2 * x * y - 2 * y * z - 2 * x * z);
}

std::vector<double> AtTools::AtKinematics::KinematicalFit(std::vector<double> &parameters)
{
   // Kinematic Fitting based on https://www.phys.ufl.edu/~avery/fitting.html
   // Input vector is composed by 4 four-momentum vectors of (projectile,ejectile,proton1,proton2)
   // Output vector contains the fitted coordinates in the same order

   Int_t rowDim = fAlphaP[0]->GetNrows();
   std::cout << " Row dimension " << rowDim << "\n";
   if (rowDim != fNumParticles * 4 || rowDim != parameters.size()) {
      std::cerr << " Wrong matrix/parameter dimension. Aborting... "
                << "\n";
      std::exit(0);
   }

   ResetMatrices();

   for (auto i = 0; i < fNumParticles; ++i) {
      for (auto j = 0; j < 4; ++j) {

         (*fAlphaP.at(i + 1))[j][0] = parameters[i * 4 + j];
      }
      (*fAlphaP.at(0)).SetSub(i, 0, (*fAlphaP.at(i + 1)));
   }

   return {};
}

void AtTools::AtKinematics::ResetMatrices()
{
   for (auto &alpha : fAlphaP)
      alpha->Zero();
}

void AtTools::AtKinematics::PrintMatrices()
{
   for (auto &alpha : fAlphaP)
      alpha->Print();
}
