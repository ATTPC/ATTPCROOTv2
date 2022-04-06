///////////////////////////////////////////////////////////
/////////Code: EnergyLoss_LookUp.h/////////////////////////
////////////Date: November 2021////////////////////////////
///////////Author: Nabin Rijal ////////////////////////////
//// This is the header file for EnergyLoss_LookUp.cpp ////
///////////////////////////////////////////////////////////
using namespace std;

class LookUp {

public:
   LookUp()
   {
      c = 29.9792458; // Speed of light in cm/ns.
      dEdx_e = 0;
      dEdx_n = 0;
      Energy_in_range = 1;
      EvD = new TGraph();
      GoodELossFile = 0;
      IonEnergy = 0;
      IonMass = 0;
      last_point = 0;
      points = 0;
      last_point1 = 0;
      points1 = 0;
   };
   ////////////////////////////////////////////////////////////////////////////////////////

   LookUp(string Eloss_file, Double_t IonMass)
   {
      Double_t IonEnergy;
      Double_t dEdx_e, dEdx_n;
      Double_t Range;
      Double_t Stragg_lon, Stragg_lat;
      string aux;

      ifstream Read(Eloss_file.c_str());

      last_point = 0;

      // cout << " Opening " << Eloss_file <<endl;
      if (!Read.is_open()) {
         cout << "*** EnergyLoss Error: File " << Eloss_file << " was not found." << endl;
         GoodELossFile = 0;
      }

      else {
         GoodELossFile = 1;
         Read >> aux >> aux >> aux >> aux >> aux >> aux; // The first line has 6 strings (columns' description).
         points = 0;                                     // Cout the number of points.

         do {
            Read >> IonEnergy >> dEdx_e >> dEdx_n >> Range >> Stragg_lon >> Stragg_lat;
            points++;
            // cout << IonEnergy << " " << dEdx_e << " " << dEdx_n << " " << points << endl ;
         } while (!Read.eof());
         // while(!Read.eof() && (points<200));
         Read.close();

         // cout << points << endl ;

         // Create the arrays depending on the number rows in the file.
         this->IonEnergy = new Double_t[points];
         this->dEdx_e = new Double_t[points];
         this->dEdx_n = new Double_t[points];
         this->Range = new Double_t[points];

         // Go to the begining of the file and read it again to now save the info in the newly created arrays.

         Read.open(Eloss_file.c_str());
         Read >> aux >> aux >> aux >> aux;

         for (int p = 0; p < points; p++) {
            Read >> IonEnergy >> dEdx_e >> dEdx_n >> Range;

            this->IonEnergy[p] = IonEnergy;
            this->dEdx_e[p] = dEdx_e;
            this->dEdx_n[p] = dEdx_n;
            this->Range[p] = Range;
         }

         Energy_in_range = 1;
         this->IonMass = IonMass; // In MeV/c^2
         c = 29.9792458;          // Speed of light in cm/ns.
         EvD = new TGraph();
      }
   };

   // LookUp(string Eloss_file, Double_t IonMass=0);
   Double_t GetEnergyLossLinear(Double_t initial_energy, Double_t distance);
   Double_t GetEnergyLoss(Double_t initial_energy, Double_t distance);
   Double_t GetInitialEnergy(Double_t FinalEnergy, Double_t PathLength, Double_t StepSize);
   Double_t GetFinalEnergy(Double_t InitialEnergy, Double_t PathLength, Double_t StepSize);
   // Double_t GetDistance(Double_t InitialE, Double_t FinalE, Double_t StepSize, int MaxSteps);
   Double_t GetDistance(Double_t InitialE, Double_t FinalE, Double_t StepSize);
   Double_t GetPathLength(Float_t InitialEnergy, Float_t FinalEnergy, Float_t DeltaT);
   Double_t LoadRange(Float_t energy1);
   Double_t GetTimeOfFlight(Double_t InitialEnergy, Double_t PathLength, Double_t StepSize);
   void SetIonMass(Double_t IonMass);
   void InitializeLookupTables(Double_t MaximumEnergy, Double_t MaximumDistance, Double_t DeltaE, Double_t DeltaD);
   void PrintLookupTables();
   Double_t GetLookupEnergy(Double_t InitialEnergy, Double_t distance);

   bool GoodELossFile;
   TGraph *EvD;

private:
   Double_t c;
   Double_t IonMass;

   Double_t *IonEnergy;
   Double_t *dEdx_e;
   Double_t *dEdx_n;
   Double_t *Range;

   Double_t MaximumEnergy;
   Double_t MaximumDistance;
   Double_t DeltaD;
   Double_t DeltaE;

   Double_t *EtoDtab;
   Double_t *DtoEtab;

   int points;
   int last_point;
   int points1;
   int last_point1;
   bool Energy_in_range;
};

//////////////////////////////////////////////////////////////////////////
class Analyzer {

public:
   Double_t ConvUtoMeV, MB1, MB2, MB3, mB1, mB2, mB3;
   Double_t La;

   Analyzer(string FileELoss1, string FileELoss2, string FileELoss3);
   ~Analyzer();
   bool SetMasses(Double_t mB1, Double_t mB2, Double_t mB3);

   // Energy_Loss pointers in the deuterium gas.
   LookUp *IonInGas1;
   LookUp *IonInGas2;
   LookUp *IonInGas3;
};
//////////////////////////////////////////////////////////////////////////
