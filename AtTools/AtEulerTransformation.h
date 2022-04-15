// -------------------------------------------------------------------------
// -----                  AtEulerTransformation header file            -----
// -----             Adapted from ActarSim (H. Alvarez-Pol)            -----
// -----   Created 04/08/15  by Y. Ayyad  (ayyadlim@nscl.msu.edu)      -----
// -------------------------------------------------------------------------
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

#ifndef AtEULERTRANSFORMAtION_H
#define AtEULERTRANSFORMAtION_H

#include <Rtypes.h>
#include <TNamed.h>

class TBuffer;
class TClass;
class TMemberInspector;

class AtEulerTransformation : public TNamed {
private:
   // inputs: teta0,phi0, beamdirection=>(alpha,beta,gamma)
   // outputs: theta_1,phi_1

   Double_t ThetaInBeamSystem{0.0};
   Double_t PhiInBeamSystem{0.0};
   Double_t BeamDirectionAtVertexTheta{0.0};
   Double_t BeamDirectionAtVertexPhi{0.0};

   Double_t ThetaInLabSystem{0.0};
   Double_t PhiInLabSystem{0.0};

public:
   AtEulerTransformation() = default;
   ~AtEulerTransformation() = default;

   void SetThetaInBeamSystem(Double_t value) { ThetaInBeamSystem = value; }
   void SetPhiInBeamSystem(Double_t value) { PhiInBeamSystem = value; }
   void SetBeamDirectionAtVertexTheta(Double_t value) { BeamDirectionAtVertexTheta = value; }
   void SetBeamDirectionAtVertexPhi(Double_t value) { BeamDirectionAtVertexPhi = value; }

   Double_t GetThetaInBeamSystem(void) { return ThetaInBeamSystem; }
   Double_t GetPhiInBeamSystem(void) { return PhiInBeamSystem; }
   Double_t GetThetaInLabSystem(void) { return ThetaInLabSystem; }
   Double_t GetPhiInLabSystem(void) { return PhiInLabSystem; }
   Double_t GetBeamDirectionAtVertexTheta(void) { return BeamDirectionAtVertexTheta; }
   Double_t GetBeamDirectionAtVertexPhi(void) { return BeamDirectionAtVertexPhi; }

   void DoTheEulerTransformationBeam2Lab();

   void Dump();

   void PrintResults();

   ClassDef(AtEulerTransformation, 1)
};
#endif
