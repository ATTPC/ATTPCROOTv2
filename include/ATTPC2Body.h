// -------------------------------------------------------------------------
// -----                   ATTPC2Body header file              -----
// -----                    Inherit from FairIonGenerator              -----
// -----                 Created 29/07/15  by Y. Ayyad                 -----
// -------------------------------------------------------------------------


#ifndef ATTPC2Body_H
#define ATTPC2Body_H


#include "FairGenerator.h"
#include "FairIon.h"
#include "FairParticle.h"

#include <iostream>
#include <map>

class FairPrimaryGenerator;
class ATTPC2Body;

class ATTPC2Body : public FairGenerator
{

 public:

  /** Default constructor **/
  ATTPC2Body();


  ATTPC2Body(const char* name,std::vector<Int_t> *z,std::vector<Int_t> *a,std::vector<Int_t> *q, Int_t mult, std::vector<Double_t> *px, 
          std::vector<Double_t>* py,std::vector<Double_t> *pz, std::vector<Double_t> *mass , std::vector<Double_t> *Ex, Double_t ResEner, Double_t MinCMSAng,Double_t MaxCMSAng);


  ATTPC2Body(const ATTPC2Body&);


  ATTPC2Body& operator=(const ATTPC2Body&) { return *this; }

  virtual Bool_t ReadEvent(FairPrimaryGenerator* primGen);


  /** Destructor **/
  virtual ~ATTPC2Body();

private:

  static Int_t fgNIon;                      //! Number of the instance of this class
  Int_t    fMult;                           // Multiplicity per event
  std::vector<Double_t> fPx, fPy, fPz;      // Momentum components [GeV] per nucleon
  std::vector<Double_t> Masses;             // Masses of the N products
  std::vector<Double_t> fExEnergy;           // Excitation energies of the products
  Double_t fVx, fVy, fVz;                   // Vertex coordinates [cm]
  std::vector<FairIon*>  fIon;              // Pointer to the FairIon to be generated
  std::vector<TString> fPType;           
  std::vector<FairParticle*>  fParticle; 
  std::vector<Int_t>   fQ;		    // Electric charge [e]
  //std::vector<Int_t> fA;
  //std::vector<Int_t> fZ;
  Double_t fBeamEnergy;			    // Residual beam energy for phase calculation
 // Double_t fBeamEnergy_buff;			    // Residual beam energy for phase calculation
 // Int_t fZBeam;
 // Int_t fABeam;
  Double_t fPxBeam;
  Double_t fPyBeam;
  Double_t fPzBeam;
  Double_t fThetaCmsMax;
  Double_t fThetaCmsMin;
  Bool_t fIsDecay;
 // Double_t fBeamMass;
 // Double_t fTargetMass;
  Bool_t fNoSolution;
  std::vector<Double_t> fWm;                                 // Total mass
 
 


  ClassDef(ATTPC2Body,1)

}; 


#endif

