// -------------------------------------------------------------------------
// -----                   AtTPC_Background header file              -----
// -----                    Inherit from FairIonGenerator       ----
// -----                 Created 17/04/17  by J. C. Zamora -----
// -------------------------------------------------------------------------

#ifndef AtTPC_Background_H
#define AtTPC_Background_H

#include <FairGenerator.h>
#include <FairIon.h>      // for FairIon
#include <FairParticle.h> // for FairParticle

#include <Rtypes.h>
#include <TString.h>

#include <memory> // for unique_ptr
#include <vector>

class FairPrimaryGenerator;
class TBuffer;
class TClass;
class TMemberInspector;

class AtTPC_Background : public FairGenerator {

public:
   /** Default constructor **/
   AtTPC_Background();

   AtTPC_Background(const char *name, std::vector<Int_t> *z, std::vector<Int_t> *a, std::vector<Int_t> *q, Int_t mult,
                    std::vector<Double_t> *px, std::vector<Double_t> *py, std::vector<Double_t> *pz,
                    std::vector<Double_t> *mass, std::vector<Double_t> *Ex);

   AtTPC_Background(const AtTPC_Background &);

   AtTPC_Background &operator=(const AtTPC_Background &) { return *this; }

   virtual Bool_t ReadEvent(FairPrimaryGenerator *primGen);

   virtual Double_t omega(Double_t x, Double_t y, Double_t z);

   virtual Double_t *TwoB(Double_t m1b, Double_t m2b, Double_t m3b, Double_t m4b, Double_t Kb, Double_t thetacm);

   virtual std::vector<Double_t>
   TRANSF(std::vector<Double_t> *from, std::vector<Double_t> *to, std::vector<Double_t> *vin);

   virtual std::vector<Double_t> BreakUp(std::vector<Double_t> *Pdeuteron);

   /** Destructor **/
   virtual ~AtTPC_Background() = default;

private:
   static Int_t fgNIon; //! Number of the instance of this class
   Int_t fMult;         // Multiplicity per event
   Bool_t fIsDecay{};
   Double_t fVx, fVy, fVz;                     // Vertex coordinates [cm]
   std::vector<std::unique_ptr<FairIon>> fIon; // Pointer to the FairIon to be generated
   std::vector<TString> fPType;
   std::vector<std::unique_ptr<FairParticle>> fParticle;
   std::vector<Double_t> fPx, fPy, fPz; // Momentum components [MeV] per nucleon
   std::vector<Double_t> Masses;        // Masses of the N products
   std::vector<Double_t> fExEnergy;     // Excitation energies of the products
   std::vector<Double_t> fWm;           // Total mass
   Int_t fN{};

   Double_t K1{};
   Double_t m1{}, m2{}, m3{}, m4{}, m7{}, m8{};
   Double_t Ex_ejectile{}, Ex_2he{};
   Double_t test_var{};

   Double_t ranX{};
   Int_t ran_theta{};

   Double_t Prec{};

   std::vector<Int_t> fQ; // Electric charge [e]

   Double_t fBeamEnergy{}; // Residual beam energy for phase calculation
   Double_t fPxBeam{};
   Double_t fPyBeam{};
   Double_t fPzBeam{};

   Double_t random_z{};
   Double_t random_r{};
   Double_t random_phi{};

   ClassDef(AtTPC_Background, 2)
};

#endif
