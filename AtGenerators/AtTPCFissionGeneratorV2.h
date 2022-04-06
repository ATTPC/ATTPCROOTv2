
#ifndef AtTPCFISSIONGENERAtORV2_H
#define AtTPCFISSIONGENERAtORV2_H

#include <Rtypes.h>
#include <RtypesCore.h>
#include <TString.h>
#include <vector>

#include <FairGenerator.h>

class FairPrimaryGenerator;
class TBuffer;
class TClass;
class TMemberInspector;
class TTree;

class AtTPCFissionGeneratorV2 : public FairGenerator {

public:
   /** Default constructor **/
   AtTPCFissionGeneratorV2();

   AtTPCFissionGeneratorV2(const char *name, TString simfile);

   AtTPCFissionGeneratorV2(const AtTPCFissionGeneratorV2 &);

   AtTPCFissionGeneratorV2 &operator=(const AtTPCFissionGeneratorV2 &) { return *this; }

   virtual Bool_t ReadEvent(FairPrimaryGenerator *primGen);

   /** Destructor **/
   virtual ~AtTPCFissionGeneratorV2();

private:
   static Int_t fgNIon; //! Number of the instance of this class
   Int_t fMult;         // Multiplicity per event
   Bool_t fIsDecay;
   Bool_t fNoSolution;

   std::vector<TTree *> pTree; // vector to contain a pointer to the tree
   Double_t fVx, fVy, fVz;     // Vertex coordinates [cm]
   Double_t fP1x, fP1y, fP1z;  // Momentum components [GeV] per nucleon
   Double_t fP2x, fP2y, fP2z;  // Momentum components [GeV] per nucleon
   Int_t Evnt;
   Int_t event;
   Int_t Aout[100], Zout[100], Ntrack;
   Double_t fOutPx[100], fOutPy[100], fOutPz[100];

   ClassDef(AtTPCFissionGeneratorV2, 1)
};

#endif
