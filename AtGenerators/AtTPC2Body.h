// -------------------------------------------------------------------------------------//
// -----                   AtTPC2Body header file                                   ----//
// -----                    Inherit from FairIonGenerator                           ----//
// -----                 Created 29/07/15  by Y. Ayyad                              ----//
// -----                 11/13/2020 Added Fixed target option for HELIOS (Y. Ayyad) ----//
// -------------------------------------------------------------------------------------//

#ifndef AtTPC2Body_H
#define AtTPC2Body_H

#include <Rtypes.h>
#include <TString.h>
#include <FairGenerator.h>
#include <vector>

class FairPrimaryGenerator;
class FairIon;
class FairParticle;
class TBuffer;
class TClass;
class TMemberInspector;

class AtTPC2Body : public FairGenerator {

public:
   /** Default constructor **/
   AtTPC2Body();

   AtTPC2Body(const char *name, std::vector<Int_t> *z, std::vector<Int_t> *a, std::vector<Int_t> *q, Int_t mult,
              std::vector<Double_t> *px, std::vector<Double_t> *py, std::vector<Double_t> *pz,
              std::vector<Double_t> *mass, std::vector<Double_t> *Ex, Double_t ResEner, Double_t MinCMSAng,
              Double_t MaxCMSAng);

   AtTPC2Body(const AtTPC2Body &);

   AtTPC2Body &operator=(const AtTPC2Body &) { return *this; }

   void SetFixedTargetPosition(double vx, double vy, double vz);
   void SetFixedBeamMomentum(double px, double py, double pz);
   inline void SetSequentialDecay(Bool_t val) { kIsDecay = val; }

   inline Bool_t GetIsDecay() { return kIsDecay; }

   virtual Bool_t ReadEvent(FairPrimaryGenerator *primGen);

   /** Destructor **/
   virtual ~AtTPC2Body();

private:
   static Int_t fgNIon;                 //! Number of the instance of this class
   Int_t fMult;                         // Multiplicity per event
   std::vector<Double_t> fPx, fPy, fPz; // Momentum components [GeV] per nucleon
   std::vector<Double_t> Masses;        // Masses of the N products
   std::vector<Double_t> fExEnergy;     // Excitation energies of the products
   Double_t fVx, fVy, fVz;              // Vertex coordinates [cm]
   std::vector<FairIon *> fIon;         // Pointer to the FairIon to be generated
   std::vector<TString> fPType;
   std::vector<FairParticle *> fParticle;
   std::vector<Int_t> fQ; // Electric charge [e]
   // std::vector<Int_t> fA;
   // std::vector<Int_t> fZ;
   Double_t fBeamEnergy;      // Residual beam energy for phase calculation
   Double_t fBeamEnergy_buff; // Residual beam energy for phase calculation
                              // Int_t fZBeam;
                              // Int_t fABeam;
   Double_t fPxBeam;
   Double_t fPyBeam;
   Double_t fPzBeam;
   Double_t fPxBeam_buff;
   Double_t fPyBeam_buff;
   Double_t fPzBeam_buff;
   Double_t fThetaCmsMax;
   Double_t fThetaCmsMin;
   Bool_t kIsDecay;
   // Double_t fBeamMass;
   // Double_t fTargetMass;
   Bool_t fNoSolution;
   std::vector<Double_t> fWm; // Total mass

   Bool_t fIsFixedTargetPos; //
   Bool_t fIsFixedMomentum;  //

   ClassDef(AtTPC2Body, 3)
};

#endif
