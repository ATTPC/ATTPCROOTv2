#ifndef ATBRAGGFITMETADATA_H
#define ATBRAGGFITMETADATA_H

#include "AtFitTrackMetadata.h"

#include <Rtypes.h> // for Double_t, THashConsistencyHolder, ClassDefOverride
#include <TObject.h>
#include <TString.h>

class TBuffer;
class TClass;
class TMemberInspector;

/**
 * Class for storing the result of the fit of an AtTrack from an AtFitter class.
 */
class AtBraggFitMetadata : public AtFitTrackMetadata {
protected:
   // Parameters of the fit and name of the ELoss model used.
   TString fELossModelName{"none"};
   Double_t fKineticEnergy{-1}; // MeV
   Double_t fKineticEnergyUncertainty{-1}; // MeV
   Double_t fAmplitudeFactor{-1}; // ADC/MeV
   Double_t fAmplitudeFactorUncertainty{-1}; // ADC/MeV

   // Particle information.
   TString fPDGCode{"none"};
   Int_t fA{0};
   Int_t fZ{0};
   Double_t fMassAmu{0};

public:
   AtBraggFitMetadata() = default;
   AtBraggFitMetadata(const AtBraggFitMetadata &) = default;
   AtBraggFitMetadata(AtBraggFitMetadata &&) = default;
   ~AtBraggFitMetadata() = default;

   void SetELossModelName(std::string name) { fELossModelName = TString(name); }
   void SetKineticEnergy(Double_t value) { fKineticEnergy = value; }
   void SetKineticEnergyUncertainty(Double_t value) { fKineticEnergyUncertainty = value; }
   void SetAmplitudeFactor(Double_t value) { fAmplitudeFactor = value; }
   void SetAmplitudeFactorUncertainty(Double_t value) { fAmplitudeFactorUncertainty = value; }
   void SetPDGCode(std::string value) { fPDGCode = TString(value); }
   void SetAtomicMassNumber(Int_t value) { fA = value; }
   void SetChargeNumber(Int_t value) { fZ = value; }
   void SetMassAmu(Double_t value) { fMassAmu = value; }

   TString GetELossModelName() { return fELossModelName; }
   Double_t GetKineticEnergy() { return fKineticEnergy; }
   Double_t GetKineticEnergyUncertainty() { return fKineticEnergyUncertainty; }
   Double_t GetAmplitudeFactor() { return fAmplitudeFactor; }
   Double_t GetAmplitudeFactorUncertainty() { return fAmplitudeFactorUncertainty; }
   TString GetPDGCode() { return fPDGCode; }
   Int_t GetAtomicMassNumber() { return fA; }
   Int_t GetChargeNumber() { return fZ; }
   Double_t GetMassAmu() { return fMassAmu; }

   virtual void Print() const override;

   ClassDefOverride(AtBraggFitMetadata, 1);
};

#endif
