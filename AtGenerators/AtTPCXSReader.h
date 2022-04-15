// -------------------------------------------------------------------------
// -----                    AtTPCXSReader header file                  -----
// -----               Created 03/07/18 by H. Alvarez Pol              -----
// -------------------------------------------------------------------------

#ifndef AtTPCXSREADER_H
#define AtTPCXSREADER_H

#include <Rtypes.h>
#include <TString.h>
#include <FairGenerator.h>
#include <vector>

class FairPrimaryGenerator;
class FairIon;
class FairParticle;
class TBuffer;
class TClass;
class TH2F;
class TMemberInspector;

class AtTPCXSReader : public FairGenerator {

public:
   /** Default constructor **/
   AtTPCXSReader();

   /** Constructor
    ** For the generation of a vertex with a given cross section.
    **/
   AtTPCXSReader(const char *name, std::vector<Int_t> *z, std::vector<Int_t> *a, std::vector<Int_t> *q, Int_t mult,
                 std::vector<Double_t> *px, std::vector<Double_t> *py, std::vector<Double_t> *pz,
                 std::vector<Double_t> *mass);

   AtTPCXSReader &operator=(const AtTPCXSReader &) { return *this; }

   /** Destructor **/
   virtual ~AtTPCXSReader() = default;

   /** Method ReadEvent
    ** Generates particles according to the XS file and send them to the
    ** FairPrimaryGenerator.
    **/
   virtual Bool_t ReadEvent(FairPrimaryGenerator *primGen);

   /** Modifiers **/
   void SetXSFileName(TString name = "xs_22Mgp_fusionEvaporation.txt") { fXSFileName = name; }

private:
   TString fXSFileName;

   static Int_t fgNIon;                 //! Number of the instance of this class
   Int_t fMult;                         // Multiplicity per event
   std::vector<Double_t> fPx, fPy, fPz; // Momentum components [GeV] per nucleon
   std::vector<Double_t> Masses;        // Masses of the N products
   std::vector<Double_t> fExEnergy;     // Excitation energies of the products
   Double_t fVx, fVy, fVz;              // Vertex coordinates [cm]
   std::vector<FairIon *> fIon;         // Pointer to the FairIon to be generated
   std::vector<TString> fPType;
   std::vector<FairParticle *> fParticle;
   std::vector<Int_t> fQ;  // Electric charge [e]
   Double_t fBeamEnergy{}; // Residual beam energy for phase calculation
   Double_t fPxBeam{};
   Double_t fPyBeam{};
   Double_t fPzBeam{};
   std::vector<Double_t> fWm; // Total mass

   TH2F *fh_pdf{};

   ClassDef(AtTPCXSReader, 1)
};

#endif
