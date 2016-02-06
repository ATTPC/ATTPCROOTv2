double mass_nucleon = 940./1000;//GeV
double mass_pi = 140./1000;//GeV
double sigma_delta_mass = 170./1000;//GeV
double beta;
double s;
double check_s;

double mass_10Be = 9327.55/1000.0; //10Be mass in GeV
double mass_4He = 3728.40/1000.0; //4He mass in GeV
double mass_6He = 5606.56/1000.0; //6He mass in GeV

TLorentzVector fEnergyImpulsionLab_beam;
TLorentzVector fEnergyImpulsionLab_target;

TLorentzVector fEnergyImpulsionLab_Total;

TLorentzVector fEnergyImpulsionFinal;

TVector3 fImpulsionLab_beam;
TVector3 fImpulsionLab_target;

Double_t mass_1[3];
Double_t mass_2[2] = {mass_nucleon,mass_pi};

TGenPhaseSpace event1;
TGenPhaseSpace event2;

TLorentzVector *pN1;
TLorentzVector *pN2;
TLorentzVector *pDelta;
TLorentzVector *pPi;

TLorentzVector *p6He;
TLorentzVector *p4He_1;
TLorentzVector *p4He_2;

//TTree *tree = new TTree("PSTree","Phase Space Tree");

double KineticEnergy6He;
double KineticEnergy4He_1;
double KineticEnergy4He_2;
double ThetaLab_6He;
double ThetaLab_4He_1;
double ThetaLab_4He_2;


double KineticEnergyN1;
double KineticEnergyDelta;
double KineticEnergyN2;
double KineticEnergyPi_Lab;
double KineticEnergyPi_CM;
double ThetaLab_Pi;
double ThetaCM_Pi;
double mass_delta;
double ThetaLab_N1;
double ThetaLab_N2;
double ThetaLab_Delta;

void InitOutput();
