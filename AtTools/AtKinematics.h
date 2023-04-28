/*********************************************************************
 *   AtTools Class for kinematics calculations                       *
 *   Author: Y. Ayyad yassid.ayyad@usc.es            	             *
 *   Log: 03/28/2022 					             *
 *								     *
 *********************************************************************/

#ifndef ATKINEMATICS_H
#define ATKINEMATICS_H
#include <Math/Vector4D.h>
#include <Math/Vector4Dfwd.h> // for PxPyPzEVector
#include <Rtypes.h>           // for Double_t, THashConsistencyHolder, Int_t, ClassDef
#include <TMatrixDfwd.h>      // for TMatrixD
#include <TMatrixT.h>         // for TMatrixT
#include <TObject.h>          // for TObject

#include <cmath>  // for sqrt
#include <memory> // for unique_ptr
#include <tuple>  // for tuple
#include <vector> // for vector
class TBuffer;
class TClass;
class TMemberInspector;

namespace AtTools {

class AtKinematics : public TObject {

private:
   Int_t fVerbosity;

   Int_t fNumParticles{3};              //
   Int_t fNumIterations{100};           //! Number of iterations for the minimizer
   Double_t fWeigth{0.05};              //! Minimization weighting
   Double_t fTargetMass{2.01410177812}; //! Mass of target for Kinematical fitting

   std::vector<std::unique_ptr<TMatrixD>> fAlphaP; //! alpha row for n particles;
   void ResetMatrices();
   void PrintMatrices();
   TMatrixD CalculateCovariance();
   TMatrixD CalculateD(TMatrixD *alpha);
   TMatrixD Calculated(TMatrixD *alpha);

   ClassDef(AtKinematics, 1);

public:
   AtKinematics();
   ~AtKinematics() = default;

   void SetVerbosity(Int_t verbosity) { fVerbosity = verbosity; }

   std::tuple<Double_t, Double_t>
   GetMomFromBrho(Double_t A, Double_t Z,
                  Double_t brho); ///< Returns momentum (in GeV) from Brho assuming M (amu) and Z;
   Double_t
   TwoBodyEx(Double_t m1, Double_t m2, Double_t m3, Double_t m4, Double_t K_proj, Double_t thetalab, Double_t K_eject);
   Double_t omega(Double_t x, Double_t y, Double_t z);

   std::vector<double> KinematicalFit(std::vector<double> &parameters);
   inline void SetKFIterations(Int_t iter) { fNumIterations = iter; }
   inline void SetKFWeighting(Double_t weight) { fWeigth = weight; }
   inline void SetKFTargetMass(Double_t mass) { fTargetMass = mass; }
};

namespace Kinematics {

double GetGamma(double KE, double m1, double m2);
double GetGamma(double beta);
double GetVelocity(double gamma);
double GetBeta(double gamma);
double GetBeta(double p, int A);
double GetBeta(double p, double mass);
double GetRelMom(double gamma, double mass);
double AtoE(double Amu);
double EtoA(double mass);

template <class Vector>
ROOT::Math::PxPyPzEVector Get4Vector(Vector mom, double m)
{
   return {mom.X(), mom.Y(), mom.Z(), std::sqrt(mom.Mag2() + m * m)};
}

} // namespace Kinematics
} // namespace AtTools

#endif
