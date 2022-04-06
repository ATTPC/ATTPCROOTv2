
#ifndef AtDIGIPAR_H
#define AtDIGIPAR_H

#include <Rtypes.h>
// FAIRROOT classes
#include <FairParGenericSet.h>
// ROOT classes
#include <TString.h>

class AtGas;
class FairLogger;
class FairParamList;
class TBuffer;
class TClass;
class TMemberInspector;

class AtDigiPar : public FairParGenericSet {
public:
   // Constructors and Destructors
   AtDigiPar(const Char_t *name, const Char_t *title, const Char_t *context);
   ~AtDigiPar();

   // Operators

   // Getters
   Int_t GetPadPlaneX();
   Int_t GetPadPlaneZ();
   Int_t GetPadSizeX();
   Int_t GetPadSizeZ();
   Int_t GetPadRows();
   Int_t GetPadLayers();
   Double_t GetAnodeWirePlaneY();
   Double_t GetGroundWirePlaneY();
   Double_t GetGatingWirePlaneY();
   AtGas *GetGas();
   Int_t GetNumTbs();
   Int_t GetTBTime();           ///< returns the time duration of a time bucket in given sampling time in ns.
   Double_t GetDriftVelocity(); ///< returns the drift velocity in cm/us.
   Double_t GetDriftLength();   ///< returns the drift length in mm
   Int_t GetYDivider();         ///< returns the slice divider

   virtual Bool_t getParams(FairParamList *paramList) override;

   TString GetFile(Int_t fileNum);

   Double_t GetBField();
   Double_t GetEField();
   Double_t GetTiltAngle();
   Int_t GetTB0();
   Double_t GetThetaLorentz();
   Int_t GetTBEntrance();
   Double_t GetZPadPlane();
   Double_t GetDensity();
   Double_t GetThetaPad();
   Double_t GetThetaRot();
   Double_t GetGasPressure();
   Double_t GetMaxRange();

   Double_t GetEIonize();
   Double_t GetFano();
   Double_t GetCoefDiffusionTrans();
   Double_t GetCoefDiffusionLong();
   Double_t GetGain();
   Double_t GetGETGain();
   Int_t GetPeakingTime();

   // Setters
   virtual void putParams(FairParamList *paramList) override;

   // Main methods

private:
   FairLogger *fLogger;

   AtGas *fGas;
   TString fGasFileName;

   Bool_t fInitialized;

   Int_t fPadPlaneX;
   Int_t fPadPlaneZ;
   Int_t fPadSizeX;
   Int_t fPadSizeZ;
   Int_t fPadRows;
   Int_t fPadLayers;
   Double_t fAnodeWirePlaneY;
   Double_t fGroundWirePlaneY;
   Double_t fGatingWirePlaneY;
   Double_t fEField;
   Int_t fNumTbs;
   Int_t fSamplingRate;
   Double_t fDriftVelocity;
   Double_t fDriftLength;
   Int_t fYDivider;
   Int_t fGasFile;
   Int_t fPadPlaneFile;
   Int_t fPadShapeFile;

   Double_t fBField;
   Double_t fTiltAng;
   Int_t fTB0;
   Double_t fThetaLorentz;
   Int_t fTBEntrance;
   Double_t fZPadPlane;
   Double_t fDensity;
   Double_t fThetaPad;
   Double_t fThetaRot;
   Double_t fGasPressure;
   Double_t fMaxRange;

   Double_t fEIonize;  //!< effective ionization energy [eV]
   Double_t fFano;     //!< Fano factor of gas
   Double_t fCoefL;    //!< longitudinal diffusion coefficient
   Double_t fCoefT;    //!< transversal diffusion coefficient
   Double_t fGain;     //!< gain factor from wire plane
   Double_t fGETGain;  //!< Gain from get electronics in fC
   Int_t fPeakingTime; //!< Peaking time of the electronics in ns

   ClassDefOverride(AtDigiPar, 2);
};

#endif
