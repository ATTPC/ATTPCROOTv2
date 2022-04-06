// -------------------------------------------------------------------------
// -----                   AtTPCFissionGenerator                       -----
// -----       Created 03/04/16 by Y. Ayyad (ayyadlim@nscl.msu.edu)    -----
// -------------------------------------------------------------------------

#ifndef AtTPCFISSIONGENERAtOR_H
#define AtTPCFISSIONGENERAtOR_H

#include <Rtypes.h>
#include <RtypesCore.h>
#include <TString.h>

#include "FairGenerator.h"

class FairPrimaryGenerator;
class TDatabasePDG;
class TBuffer;
class TClass;
class TMemberInspector;
class TTree;

class AtTPCFissionGenerator : public FairGenerator {

public:
   AtTPCFissionGenerator();
   AtTPCFissionGenerator(const char *name, TString simfile);

   AtTPCFissionGenerator(const AtTPCFissionGenerator &);

   AtTPCFissionGenerator &operator=(const AtTPCFissionGenerator &) { return *this; }

   virtual Bool_t ReadEvent(FairPrimaryGenerator *primGen);

   virtual ~AtTPCFissionGenerator();

private:
   TTree *fTree;
   Double_t fVx, fVy, fVz;    // Vertex coordinates [cm]
   Double_t fP1x, fP1y, fP1z; // Momentum components [GeV] per nucleon
   Double_t fP2x, fP2y, fP2z; // Momentum components [GeV] per nucleon
   Int_t Evnt;
   Int_t event;
   Int_t Aout[100], Zout[100], Ntrack;
   Float_t fOutPx[100], fOutPy[100], fOutPz[100];

   TDatabasePDG *fPDG; //!  PDG database

   /*std::ifstream*  fInputFilebase;
   TString fFileNamebase;
   Int_t RegisterIons();
   std::map<TString, FairIon*> fIonMap;       //!*/

   ClassDef(AtTPCFissionGenerator, 5)
};

#endif
