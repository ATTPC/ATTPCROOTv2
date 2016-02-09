// -------------------------------------------------------------------------
// -----                  ATEulerTransformation header file            -----
// -----             Adapted from ActarSim (H. Alvarez-Pol)            -----
// -----   Created 04/08/15  by Y. Ayyad  (ayyadlim@nscl.msu.edu)      -----
// -------------------------------------------------------------------------

#ifndef ATEULERTRANSFORMATION_H
#define ATEULERTRANSFORMATION_H 

#include "TNamed.h"
#include "TObject.h"


class ATEulerTransformation : public TNamed 
{
private:

// inputs: teta0,phi0, beamdirection=>(alpha,beta,gamma)
// outputs: theta_1,phi_1

  Double_t ThetaInBeamSystem;
  Double_t PhiInBeamSystem;
  Double_t BeamDirectionAtVertexTheta;
  Double_t BeamDirectionAtVertexPhi;

  Double_t ThetaInLabSystem;
  Double_t PhiInLabSystem;

public:
  ATEulerTransformation();
  ~ATEulerTransformation();

  void SetThetaInBeamSystem(Double_t value){ThetaInBeamSystem = value;}
  void SetPhiInBeamSystem(Double_t value){PhiInBeamSystem = value;}
  void SetBeamDirectionAtVertexTheta(Double_t value){BeamDirectionAtVertexTheta = value;}
  void SetBeamDirectionAtVertexPhi(Double_t value){BeamDirectionAtVertexPhi = value;}

  Double_t GetThetaInBeamSystem(void){return ThetaInBeamSystem;}
  Double_t GetPhiInBeamSystem(void){return PhiInBeamSystem;}
  Double_t GetThetaInLabSystem(void){return ThetaInLabSystem;}
  Double_t GetPhiInLabSystem(void){return PhiInLabSystem;}
  Double_t GetBeamDirectionAtVertexTheta(void){return BeamDirectionAtVertexTheta;}
  Double_t GetBeamDirectionAtVertexPhi(void){return BeamDirectionAtVertexPhi;}

  void DoTheEulerTransformationBeam2Lab();

  void Dump();

  void PrintResults();

  ClassDef(ATEulerTransformation,1)

};
#endif
