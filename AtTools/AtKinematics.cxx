#include "AtKinematics.h"

#include <Rtypes.h>        // for TGenericClassInfo
#include <TError.h>        // for Error
#include <TMath.h>         // for Power, Sqrt
#include <TMatrixTUtils.h> // for TMatrixTRow

#include <algorithm> // for max
#include <cassert>   // for assert
#include <cmath>     // for pow, sqrt, cos
#include <cstdlib>   // for exit
#include <iostream>  // for operator<<, basic_ostream, basic_ostream<>::__os...
#include <utility>   // for mov
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

   std::vector<Double_t> parOut;

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
      (*fAlphaP.at(0)).SetSub(i * 4, 0, (*fAlphaP.at(i + 1)));
   }

   TMatrixD *alpha;
   TMatrixD dalpha(4 * fNumParticles, 1); // delta alpha
   dalpha.Zero();
   alpha = fAlphaP.at(0).get();

   // alpha->Print();

   TMatrixD chi2(1, 1);
   chi2.Zero();

   TMatrixD Cov = CalculateCovariance();

   double weighting = 0.05;

   for (auto i = 0; i < fNumIterations; i++) {

      TMatrixD D = CalculateD(alpha);
      TMatrixD Dt = D;
      Dt.T();
      TMatrixD Vd = D * Cov * Dt;
      Vd.Invert();
      Cov = Cov - Cov * Dt * Vd * D * Cov * weighting;

      // calculate labmda
      TMatrixD d = Calculated(alpha);
      TMatrixD temp1 = (d + D * dalpha);
      TMatrixD lambda = Vd * temp1;

      // calculate chi2
      TMatrixD lambdaT = lambda;
      chi2 = lambdaT.T() * temp1;

      // calculate alpha
      *alpha = *fAlphaP.at(0).get() - weighting * Cov * Dt * lambda;
      dalpha = *alpha - *fAlphaP.at(0).get();
      *fAlphaP.at(0).get() = *alpha;

      // std::cout<<" chi2 "<<chi2[0][0]<<"\n";

      if (fabs(chi2[0][0]) < 1.0)
         break;
   }

   for (auto i = 0; i < 4 * fNumParticles; ++i)
      parOut.push_back((*alpha)[i][0]);

   return parOut;
}

TMatrixD AtTools::AtKinematics::Calculated(TMatrixD *alpha)
{
   TMatrixD dval(4, 1);
   dval.Zero();
   double mt = fTargetMass * 931.494; // target mass

   double mout1 = 0.0;
   double mout2 = 0.0;
   double mout3 = 0.0;
   double mout4 = 0.0;

   // Exit channel momentum
   for (auto i = 1; i < fNumParticles; ++i) {
      mout1 += (*alpha)[4 * i][0];
      mout2 += (*alpha)[4 * i + 1][0];
      mout3 += (*alpha)[4 * i + 2][0];
      mout4 += (*alpha)[4 * i + 3][0];
      // std::cout<<mout1<<"\n";
   }

   double h1 = (*alpha)[0][0] - mout1;      // momentum conservation X
   double h2 = (*alpha)[1][0] - mout2;      // momentum conservation Y
   double h3 = (*alpha)[2][0] - mout3;      // momentum conservation Z
   double h4 = (*alpha)[3][0] + mt - mout4; // Total Energy conservation

   dval[0][0] = h1;
   dval[1][0] = h2;
   dval[2][0] = h3;
   dval[3][0] = h4;

   // dval.Print();

   return dval;
}

TMatrixD AtTools::AtKinematics::CalculateD(TMatrixD *alpha)
{

   TMatrixD Dval(4, 4 * fNumParticles);
   Dval.Zero();

   for (auto i = 0; i < fNumParticles; ++i) {
      TMatrixD Dsub(4, 4);
      Dsub.Zero();

      Dsub[0][0] = 1;
      for (int j = 1; j < 4; j++)
         Dsub[j][j] = -1;

      Dval.SetSub(0, i * 4, Dsub);
   }

   // Dval.Print();

   return Dval;
}

TMatrixD AtTools::AtKinematics::CalculateCovariance()
{
   TMatrixD Vval(4 * fNumParticles, 4 * fNumParticles);
   Vval.Zero();

   for (auto i = 0; i < fNumParticles; ++i) {
      TMatrixD Vsub(4, 4);
      Vsub.Zero();

      for (int j = 0; j < 4; j++)
         Vsub[j][j] = 0.1;

      Vval.SetSub(i * 4, i * 4, Vsub);
   }

   // Vval.Print();

   return Vval;
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

namespace AtTools::Kinematics {

/**
 * Get gamma for fragment 1 in a system decaying into two fragments with total KE
 * Units are SR (c=1).
 */
double GetGamma(double KE, double m1, double m2)
{
   double num = KE * KE + 2 * (m1 + m2) * (KE + m1);
   double denom = 2 * m1 * (KE + m1 + m2);
   return num / denom;
}

/**
 * Get velocity (cm/ns) of a particle with gamma.
 */
double GetVelocity(double gamma)
{
   return GetBeta(gamma) * TMath::C() * 1e-7;
}

/**
 * Get velocity (SR units, c=1) of a particle with gamma.
 */
double GetBeta(double gamma)
{
   return std::sqrt(gamma * gamma - 1) / gamma;
}
/**
 * Get velocity (SR units, c=1) of a particle with momentum p (MeV) and mass m (MeV).
 */
double GetBeta(double p, double m)
{
   return p / std::sqrt(p * p + m * m);
}
double GetBeta(double p, int A)
{
   return GetBeta(p, AtoE(A));
}

double GetGamma(double beta)
{
   assert(beta >= 0 && beta <= 1);
   return 1 / std::sqrt(1 - beta * beta);
}

/**
 * Get the relativistic momentum of a particle will mass (MeV)
 */
double GetRelMom(double gamma, double mass)
{
   return std::sqrt(gamma * gamma - 1) * mass;
}

/**
 * Get the mass in MeV of a fragment of mass in amu (or A)
 */
double AtoE(double Amu)
{
   return Amu * 931.5;
}
double EtoA(double mass)
{
   return mass / 931.5;
}

} // namespace AtTools::Kinematics
