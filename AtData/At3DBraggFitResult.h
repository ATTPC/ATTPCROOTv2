#ifndef AT3DBRAGGFITRESULT_H
#define AT3DBRAGGFITRESULT_H

#include <Rtypes.h>
#include <TObject.h>

#include <iostream>
#include <memory>
#include <utility>
#include <vector>

class At3DBraggFitResult : public TObject {
private:
   std::vector<std::pair<Int_t, Int_t>> fAZPairs{};
   std::vector<Double_t> fKineticEnergies{};
   std::vector<Double_t> fAmplitudeFactors{};
   std::vector<Double_t> fChi2s;
   std::vector<std::vector<std::pair<Double_t, Double_t>>> fELossVectors{};
   // std::vector<std::vector<std::pair<Double_t, Double_t>>> fDeDxVectors{};

   Int_t fBestFitIndex{-1};

public:
   At3DBraggFitResult() = default;
   At3DBraggFitResult(const At3DBraggFitResult &) = default;
   ~At3DBraggFitResult() = default;

   std::unique_ptr<At3DBraggFitResult> Clone() const { return std::make_unique<At3DBraggFitResult>(*this); }

   // Setters and adders.
   void AddAZPair(std::pair<Int_t, Int_t> &pair) { fAZPairs.push_back(pair); }
   void AddKineticEnergy(Double_t &energy) { fKineticEnergies.push_back(energy); }
   void AddAmplitudeFactor(Double_t &amplitude) { fAmplitudeFactors.push_back(amplitude); }
   void AddChi2(Double_t &chi2) { fChi2s.push_back(chi2); }
   void AddELoss(std::vector<std::pair<Double_t, Double_t>> &ELoss) { fELossVectors.push_back(ELoss); }
   // void AddDeDx(std::vector<std::pair<Double_t, Double_t>> &DeDx) { fDeDxVectors.push_back(DeDx); }

   void SetBestFitIndex(Int_t &index);

   // Getters.
   std::pair<Int_t, Int_t> GetAZPair(Int_t index) const { return fAZPairs[index]; }
   Double_t GetKineticEnergy(Int_t index) const { return fKineticEnergies[index]; }
   Double_t GetAmplitudeFactor(Int_t index) const { return fAmplitudeFactors[index]; }
   Double_t GetChi2(Int_t index) const { return fChi2s[index]; }
   std::vector<std::pair<Double_t, Double_t>> GetELoss(Int_t index) const { return fELossVectors[index]; }
   // std::vector<std::pair<Double_t, Double_t>> GetDeDx(Int_t index) const { return fDeDxVectors[index]; }

   Int_t GetBestFitIndex() const { return fBestFitIndex; }

   Int_t GetNumberOfEntries() const { return fChi2s.size(); }

   ClassDef(At3DBraggFitResult, 1);
};

#endif
