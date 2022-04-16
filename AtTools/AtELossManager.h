///////////////////////////////////////////////////////////
/////////Code: EnergyLoss LookUp/////////////////////////
////////////Date: November 2021////////////////////////////
///////////Author: Nabin Rijal ////////////////////////////
///////////////////////////////////////////////////////////

#ifndef AtELOSSMANAGER_H
#define AtELOSSMANAGER_H

#include <Rtypes.h>
#include <TObject.h>

#include <memory>
#include <string>
#include <vector>

class TBuffer;
class TClass;
class TGraph;
class TMemberInspector;

namespace AtTools {

class AtELossManager : public TObject {

public:
   AtELossManager();
   AtELossManager(std::string Eloss_file, Double_t Mass);
   ~AtELossManager();

   Double_t GetEnergyLossLinear(Double_t energy, Double_t distance);
   Double_t GetEnergyLoss(Double_t energy, Double_t distance);
   Double_t GetInitialEnergy(Double_t FinalEnergy, Double_t PathLength, Double_t StepSize);
   Double_t GetFinalEnergy(Double_t InitialEnergy, Double_t PathLength, Double_t StepSize);
   Double_t GetDistance(Double_t InitialE, Double_t FinalE, Double_t StepSize);
   Double_t GetPathLength(Float_t InitialEnergy, Float_t FinalEnergy, Float_t DeltaT);
   Double_t LoadRange(Float_t energy1);
   Double_t GetTimeOfFlight(Double_t InitialEnergy, Double_t PathLength, Double_t StepSize);
   void SetIonMass(Double_t IonMass);
   void InitializeLookupTables(Double_t MaximumEnergy, Double_t MaximumDistance, Double_t DeltaE, Double_t DeltaD);
   void PrintLookupTables();
   Double_t GetLookupEnergy(Double_t InitialEnergy, Double_t distance);

private:
   std::shared_ptr<TGraph> EvD;

   Double_t c{29.9792458};
   Double_t IonMass{0};

   std::vector<Double_t> IonEnergy;
   std::vector<Double_t> dEdx_e;
   std::vector<Double_t> dEdx_n;
   std::vector<Double_t> Range;

   Double_t fMaximumEnergy{};
   Double_t fMaximumDistance{};
   Double_t fDeltaD{};
   Double_t fDeltaE{};

   std::vector<Double_t> EtoDtab;
   std::vector<Double_t> DtoEtab;

   Int_t points{0};
   Int_t last_point{0};
   Int_t points1{0};
   Int_t last_point1{0};
   Bool_t Energy_in_range{true};
   Bool_t GoodELossFile{false};

   ClassDef(AtELossManager, 1)
};
} // namespace AtTools

#endif
