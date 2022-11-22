// -------------------------------------------------------------------------
// -----                   AtTPC_d2He header file              -----
// -----                    Inherit from FairIonGenerator       ----
// -----                 Created 29/08/16  by J. C. Zamora -----
// -------------------------------------------------------------------------

#ifndef AtTPC_d2He_H
#define AtTPC_d2He_H

#include <FairGenerator.h>

#include <Rtypes.h>
#include <TString.h>

#include <vector>

class FairPrimaryGenerator;
class FairIon;
class FairParticle;
class TBuffer;
class TClass;
class TMemberInspector;

class AtTPC_d2He : public FairGenerator {

public:
   /** Default constructor **/
   AtTPC_d2He();

   AtTPC_d2He(const char *name, std::vector<Int_t> *z, std::vector<Int_t> *a, std::vector<Int_t> *q, Int_t mult,
              std::vector<Double_t> *px, std::vector<Double_t> *py, std::vector<Double_t> *pz,
              std::vector<Double_t> *mass, std::vector<Double_t> *Ex, std::vector<Double_t> *cross1,
              std::vector<Double_t> *cross2, std::vector<Double_t> *cross3, Int_t N_data);

   AtTPC_d2He(const AtTPC_d2He &);

   AtTPC_d2He &operator=(const AtTPC_d2He &) { return *this; }

   virtual Bool_t ReadEvent(FairPrimaryGenerator *primGen);
   virtual std::vector<Double_t>
   TRANSF(std::vector<Double_t> *from, std::vector<Double_t> *to, std::vector<Double_t> *vin);
   virtual Double_t omega(Double_t x, Double_t y, Double_t z);
   Int_t goodEvts{};
   Int_t badEvts{};

   /** Destructor **/
   virtual ~AtTPC_d2He() = default;

private:
   static Int_t fgNIon; //! Number of the instance of this class
   Int_t fMult;         // Multiplicity per event
   Bool_t fIsDecay{};
   Double_t fVx, fVy, fVz;      // Vertex coordinates [cm]
   std::vector<FairIon *> fIon; // Pointer to the FairIon to be generated
   std::vector<TString> fPType;
   std::vector<FairParticle *> fParticle;
   std::vector<Double_t> fPx, fPy, fPz; // Momentum components [MeV] per nucleon
   std::vector<Double_t> Masses;        // Masses of the N products
   std::vector<Double_t> fExEnergy;     // Excitation energies of the products
   std::vector<Double_t> fWm;           // Total mass
   Int_t fN{};
   Double_t fCStot{};

   Double_t K1{};
   Double_t m1{}, m2{}, m3{}, m4{}, m7{}, m8{};
   Double_t Ex_ejectile{}, Ex_2he{};
   Double_t test_var{};

   Double_t beta_cm{};
   Double_t gamma_cm{};
   Double_t beta4{}, gamma4{};
   Double_t S{}, S_78{};
   Double_t Pcm{}, Pc78{};
   Double_t normP4L{};
   Double_t E1L{};
   Double_t E3C{}, E4C{}, E3L{}, E4L{};
   Double_t E7rest{}, E8rest{};
   Double_t E7L{}, E8L{};
   Double_t p1L[3]{}, p3L[3]{}, p4L[3]{};
   Double_t p3C[3]{}, p4C[3]{};
   Double_t p7rest[3]{}, p8rest[3]{};
   Double_t p7L[3]{}, p8L[3]{};
   // Double_t theta78, phi78;
   Double_t ran1{}, ran2{}, ranX{};
   Int_t ran_theta{};
   std::vector<Double_t> fvfrom, fvto, fvin, fvout;
   std::vector<Double_t> inp1, inp2, inp3;
   Double_t theta_cm{}, phi_cm{}, epsilon{};
   Double_t theta78{}, phi78{};

   std::vector<Int_t> fQ; // Electric charge [e]

   Double_t fBeamEnergy{}; // Residual beam energy for phase calculation
   // Int_t fZBeam;
   // Int_t fABeam;
   Double_t fPxBeam{};
   Double_t fPyBeam{};
   Double_t fPzBeam{};

   Double_t random_z{};
   Double_t random_r{};
   Double_t random_phi{};

   // Double_t fBeamMass;
   // Double_t fTargetMass;

   ClassDef(AtTPC_d2He, 2)
};

#endif
