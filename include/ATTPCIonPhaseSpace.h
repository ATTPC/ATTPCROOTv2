// -------------------------------------------------------------------------
// -----                   ATTPCIonPhaseSpace header file              -----
// -----                    Inherit from FairIonGenerator              -----
// -----                 Created 14/07/15  by Y. Ayyad                 -----
// -------------------------------------------------------------------------


#ifndef ATTPCIonPhaseSpace_H
#define ATTPCIonPhaseSpace_H


#include "FairGenerator.h"
#include "FairIon.h"

#include <iostream>
#include <map>

class FairPrimaryGenerator;
class ATTPCIonGenerator;

class ATTPCIonPhaseSpace : public FairGenerator
{

 public:

  /** Default constructor **/
  ATTPCIonPhaseSpace();



  /** Default constructor 
   ** For the generation of ions with atomic number z and mass number a.
   ** By default, the mass equals a times the proton mass and the 
   ** excitation energy is zero. This can be changed with the
   ** respective modifiers.
   **@param z         Atomic number
   **@param a         Atomic mass
   **@param q         Electric charge [e]
   **@param mult      Number of ions per event
   **@param px,py,pz  Momentum components [GeV] per nucleon!
   **@param vx,vy,vz  Vertex coordinates [cm]
   **/
  ATTPCIonPhaseSpace(const char* name,std::vector<Int_t> *z,std::vector<Int_t> *a,std::vector<Int_t> *q, Int_t mult, std::vector<Double_t> *px, 
          std::vector<Double_t>* py,std::vector<Double_t> *pz, std::vector<Double_t> *mass , Double_t ResEner, Int_t ZB, Int_t AB, Double_t PxB, Double_t PyB, Double_t PzB, Double_t BMass, Double_t TMass);


  ATTPCIonPhaseSpace(const ATTPCIonPhaseSpace&);


  ATTPCIonPhaseSpace& operator=(const ATTPCIonPhaseSpace&) { return *this; }

  virtual Bool_t ReadEvent(FairPrimaryGenerator* primGen);


  /** Destructor **/
  virtual ~ATTPCIonPhaseSpace();

private:

  static Int_t fgNIon;                      //! Number of the instance of this class
  Int_t    fMult;                           // Multiplicity per event
  std::vector<Double_t> fPx, fPy, fPz;      // Momentum components [GeV] per nucleon
  std::vector<Double_t> Masses;             // Masses of the N products
  Double_t fVx, fVy, fVz;                   // Vertex coordinates [cm]
  std::vector<FairIon*>  fIon;              // Pointer to the FairIon to be generated
  std::vector<Int_t>   fQ;		    // Electric charge [e]
  //std::vector<Int_t> fA;
  //std::vector<Int_t> fZ;
  Double_t fBeamEnergy;			    // Residual beam energy for phase calculation
  Double_t fBeamEnergy_buff;			    // Residual beam energy for phase calculation
  Int_t fZBeam;
  Int_t fABeam;
  Double_t fPxBeam;
  Double_t fPyBeam;
  Double_t fPzBeam;
  Bool_t fIsDecay;
  Double_t fBeamMass;
  Double_t fTargetMass;
 


  ClassDef(ATTPCIonPhaseSpace,2)

}; 


#endif

