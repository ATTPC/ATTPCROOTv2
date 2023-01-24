
#ifndef AtDIGIPAR_H
#define AtDIGIPAR_H

#include <FairParGenericSet.h>

#include <Rtypes.h>

class FairParamList;
class TBuffer;
class TClass;
class TMemberInspector;

class AtDigiPar : public FairParGenericSet {
private:
   Bool_t fInitialized;

   Double_t fBField{};
   Double_t fEField{};

   // Detector geometry
   Int_t fTBEntrance{};
   Double_t fZPadPlane{};

   // Gas properties
   Double_t fEIonize{};       //< effective ionization energy [eV]
   Double_t fFano{};          //< Fano factor of gas
   Double_t fCoefL{};         //< longitudinal diffusion coefficient [cm^2/us]
   Double_t fCoefT{};         //< transversal diffusion coefficient [cm^2/us]
   Double_t fGasPressure{};   //< gas pressure [torr]
   Double_t fDensity{};       //< Gas density [kg/m^3]
   Double_t fDriftVelocity{}; //< Electron drift velocity [cm/us]
   Double_t fGain{};          //< gain factor from wire plane

   // Electronic info
   Int_t fSamplingRate{};
   Double_t fGETGain{};  //< Gain from get electronics in fC
   Int_t fPeakingTime{}; //< Peaking time of the electronics in ns

public:
   // Constructors and Destructors
   AtDigiPar(const Char_t *name, const Char_t *title, const Char_t *context);
   ~AtDigiPar() = default;

   // Getters
   Double_t GetBField() const { return fBField; }
   Double_t GetEField() const { return fEField; }

   Int_t GetTBEntrance() const { return fTBEntrance; }
   Double_t GetZPadPlane() const { return fZPadPlane; }

   Double_t GetEIonize() const { return fEIonize; }
   Double_t GetFano() const { return fFano; }
   Double_t GetCoefDiffusionTrans() const { return fCoefT; }
   Double_t GetCoefDiffusionLong() const { return fCoefL; }
   Double_t GetGasPressure() const { return fGasPressure; }
   Double_t GetDensity() const { return fDensity; }
   Double_t GetDriftVelocity() const { return fDriftVelocity; }
   Double_t GetGain() const { return fGain; }

   Int_t GetTBTime() const;
   Double_t GetGETGain() const { return fGETGain; };
   Int_t GetPeakingTime() const { return fPeakingTime; };

   // Setters
   virtual void putParams(FairParamList *paramList) override;
   virtual Bool_t getParams(FairParamList *paramList) override;
   // Main methods

   ClassDefOverride(AtDigiPar, 4);
};

#endif
