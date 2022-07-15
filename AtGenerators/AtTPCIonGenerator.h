// -------------------------------------------------------------------------
// -----                   AtTPCIonGenerator header file               -----
// -----                    Based on FairIonGenerator                  -----
// -----                 Created 30/01/15  by Y. Ayyad                 -----
// -------------------------------------------------------------------------

/** Include beam spot size **/

#ifndef AtTPCIONGENERAtOR_H
#define AtTPCIONGENERAtOR_H

#include <FairGenerator.h>

#include <TH1F.h>
#include <Rtypes.h>

class FairPrimaryGenerator;
class FairIon;
class TBuffer;
class TClass;
class TMemberInspector;

class AtTPCIonGenerator : public FairGenerator {
private:
   void SetEmittance();
   void SetBeamOrigin();

   static Int_t fgNIon;        //! Number of the instance of this class
   Int_t fMult;                //< Multiplicity per event
   Double_t fPx, fPy, fPz;     //< Momentum components [GeV] per nucleon
   Double32_t fR, fz, fOffset; //< beam Spot radius [cm], z source, y source
   Double_t fVx, fVy, fVz;     //< Vertex coordinates [cm]
   Double_t fTheta{0};         //< Maximum angle [rad]
   FairIon *fIon;              //< Pointer to the FairIon to be generated
   Int_t fQ;                   //< Electric charge [e]
   Int_t fNomEner{};
   Double_t fMaxEnLoss{}; //< Max energy loss before reation happens
   Double32_t fWhmFocus, fDiv, fZFocus,
      fRHole; //< Beam whm at focus, beam divergence, z focus, radius of the pad plan hole

   Int_t fBeamOpt;
   Bool_t fDoReact{true};

public:
   /** Default constructor **/
   AtTPCIonGenerator();

   /** Constructor with ion name
    ** For the generation of ions with pre-defined FairIon
    ** By default, the  excitation energy is zero. This can be changed with the
    ** respective modifiers.
    **@param ionName  Ion name
    **@param mult      Number of ions per event
    **@param px,py,pz  Momentum components [GeV] per nucleon!
    **@param vx,vy,vz  Vertex coordinates [cm]
    **/
   AtTPCIonGenerator(const Char_t *ionName, Int_t mult, Double_t px, Double_t py, Double_t pz);

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
    **@param ener      Energy of the ion.
    **@param eLoss     Maximum energy loss before reaction happens. Defaults to ener.
    **/
   // AtTPCIonGenerator(const char *name, Int_t z, Int_t a, Int_t q, Int_t mult, Double_t px, Double_t py, Double_t pz,
   //                   Double_t Ex, Double_t m, Double_t ener, Double_t eLoss = -1);

   AtTPCIonGenerator(const char* name,Int_t z, Int_t a, Int_t q, Int_t mult, Double_t px,
	                   Double_t py, Double_t pz, Double_t Ex, Double_t m, Double_t ener,
                   	 Double_t eLoss = -1, TString sata="", TString sbta="");

   AtTPCIonGenerator(const AtTPCIonGenerator &);

   /** Destructor **/
   virtual ~AtTPCIonGenerator() = default;

   /** Modifiers **/
   void SetCharge(Int_t charge) { fQ = charge; }
   void SetExcitationEnergy(Double_t eExc);
   void SetMass(Double_t mass);

   void SetSpotRadius(Double32_t r = 0, Double32_t z = 0, Double32_t off = 0)
   {
      fR = r;
      fz = z;
      fOffset = off;
      fBeamOpt = 1;
   }

   void SetBeamLimits(Double32_t r = 0, Double32_t z = 0, Double_t theta = 0)
   {
      fR = r;
      fz = z;
      fTheta = theta;
      fBeamOpt = 3;
   }

   void SetBeamEmittance(Double32_t val1=0, Double32_t val2=0, Double32_t val3=0, Double32_t val4=0, Double_t val5=0,
     Double_t val6=0, Double_t val7=0, Double_t val8=0, Double_t val9=0)
   {
      fWhmFocus=val1;
      fDiv=val2;
      fZFocus=val3;
      fRHole=val4;
      fmomAcc=val5;
      fBeamAx=val6;
      fBeamAy=val7;
      fBeamOx=val8;
      fBeamOy=val9;
      fBeamOpt=2;
   }

   void SetDoReaction(Bool_t doReact) { fDoReact = doReact; }

   /** Method ReadEvent
   ** Generates <mult> of the specified ions and hands hem to the
   ** FairPrimaryGenerator.
   **/
   virtual Bool_t ReadEvent(FairPrimaryGenerator *primGen);

<<<<<<< HEAD
=======
private:
   void SetEmittance();

   static Int_t fgNIon;        //! Number of the instance of this class
   Int_t fMult;                // Multiplicity per event
   Double_t fPx, fPy, fPz;     // Momentum components [GeV] per nucleon
   Double_t fPx0, fPy0, fPz0;   // initial Momentum components [GeV] per nucleon, used for momentum acceptance
   Double32_t fR, fz, fOffset; // beam Spot radius [cm], z source, y source
   Double_t fVx, fVy, fVz;     // Vertex coordinates [cm]
   FairIon *fIon;              // Pointer to the FairIon to be generated
   Int_t fQ;                   // Electric charge [e]
   Int_t fNomEner{};
   Double_t fMaxEnLoss{}; // Max energy loss before reation happens
   Double32_t fWhmFocus, fDiv, fZFocus,
      fRHole; // Beam whm at focus, beam divergence, z focus, radius of the pad plan hole
   Double_t fmomAcc; // beam momentum acceptance in percentage
   Double_t fBeamAx, fBeamAy; // beam angle [deg]
   Double_t fBeamOx, fBeamOy; // beam offset [cm]
   TH1F *hAta,*hBta;// beam angles distributions in dispersive and non dispersive direction (respectively to S800) [rad]
   Int_t fBeamOpt;

>>>>>>> modif to run the d,2He simu, digi and analysis
   ClassDef(AtTPCIonGenerator, 1)
};

#endif
