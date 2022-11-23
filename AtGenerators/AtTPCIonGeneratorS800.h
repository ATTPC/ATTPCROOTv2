#ifndef AtTPCIONGENERAtORS800_H
#define AtTPCIONGENERAtORS800_H

#include "AtTPCIonGenerator.h"

#include <Rtypes.h>  // for Double_t, Double32_t, Int_t, THashCon...
#include <TH1.h>     // for TH1F
#include <TString.h> // for TString

#include <memory> // for unique_ptr

class TBuffer;
class TClass;
class TMemberInspector;

class AtTPCIonGeneratorS800 : public AtTPCIonGenerator {
protected:
   virtual void SetVertexCoordinates() override;

   Double_t fPx0{0}, fPy0{0}, fPz0{0}; //< initial Momentum components [GeV] per nucleon, used for momentum acceptance

   Double_t fMomAcc{0};             //< beam momentum acceptance in percentage
   Double_t fBeamAx{0}, fBeamAy{0}; //< beam angle [deg]
   Double_t fBeamOx{0}, fBeamOy{0}; //< beam offset [cm]
   /// beam angles distributions in dispersive and non dispersive direction(respectively to S800)[rad]
   std::unique_ptr<TH1F> fAta, fBta;
   /// Beam whm at focus, beam divergence, z focus, radius of the pad plan hole
   Double32_t fWhmFocus{0}, fDiv{0}, fZFocus{0}, fRHole{0};

public:
   AtTPCIonGeneratorS800();
   AtTPCIonGeneratorS800(const char *name, Int_t z, Int_t a, Int_t q, Int_t mult, Double_t px, Double_t py, Double_t pz,
                         Double_t Ex, Double_t m, Double_t ener, Double_t eLoss = -1, TString sata = "",
                         TString sbta = "");

   /*
      virtual ~AtTPCIonGeneratorS800()
      {
         delete fileAta;
         delete fileBta;
      }
   */
   void SetBeamEmittance(Double32_t val1 = 0, Double32_t val2 = 0, Double32_t val3 = 0, Double32_t val4 = 0,
                         Double_t val5 = 0, Double_t val6 = 0, Double_t val7 = 0, Double_t val8 = 0, Double_t val9 = 0);

   ClassDefOverride(AtTPCIonGeneratorS800, 1);
};

#endif
