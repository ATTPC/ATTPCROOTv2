#ifndef TRelativisticDecay_h
#define TRelativisticDecay_h 1

#include <TH2.h>
#include <TStyle.h>
#include <TCanvas.h>
#include <iostream>
#include <TCanvas.h>
#include <TColor.h>
#include <TH1.h>
#include <TLegend.h>
#include <TRandom.h>
#include <TCutG.h>
#include <TProfile.h>
#include <TMath.h>
#include <TF1.h>

using namespace std;

class TRelativisticDecay{
	
	private:
	
	double m1; //mass0 of the mother nucleus
	
	double m3; //mass0 of the daughter nucleus 1
	double m4; //mass0 of the daughter nucleus 2
	double ex1; //excitation energy of mother nuclei
	
	double ex3; //excitation energy of the daughter nuclei 1
	double ex4; //excitation energy of the daughter nuclei 2
	
	double ebInput; // incident energy (total Lab energy in MeV), here this is the kinetic energy of the unbound recoil
	
	double thetacm1Input; // Theta CM angle of the daughter nuclei
	
	double ebInput; // Energy of the recoil produced in two body collision
	
	double* ANGA1; // lab angle and energy of daughter nuclei 1
	double* ANGA2; // lab angle and energy of daughter nuclei 2
	bool    NoSolution;
	
	
	
	public:
	
	TRelativisticDecay();
	~TRelativisticDecay();
	
	void SetMassOfMother(double value){m1 = value;}
	void SetMassOfDaughter1(double value){m3 = value;}
	void SetMassOfDaughter2(double value){m4 = value;}
	void SetExEnergyOfMother(double value){ex1 = value;}
	void SetExEnergyOfDaughter1(double value){ex3 = value;}
	void SetExEnergyOfDaughter2(double value){ex4 = value;}
	
	void SetLabEnergyDecay(double value){ebInput = value;}
	
	void SetThetaCMAngleDecay(double value){thetacm1Input = value;}
	
	void SetANGA1(int place, double value){ANGA1[place] = value;}
	void SetANGA2(int place, double value){ANGA2[place] = value;}
	
	void SetNoSolution(bool value){NoSolution=value;}
	
	double GetMassOfMother(void){return m1;}
	double GetMassOfDaughter1(void){return m3;}
	double GetMassOfDaughter2(void){return m4;}
	double GetExEnergyOfMother(void){return ex1;}
	double GetExEnergyOfDaughter1(void){return ex3;}
	double GetExEnergyOfDaughter2(void){return ex4;}
	
	double GetLabEnergy(void){return tb;}
	
	double GetThetaCMAngle(void){return thetacm1Input;}
	
	double GetANGA1(int place){return ANGA1[place];}
	double GetANGA2(int place){return ANGA2[place];}
	
	bool GetNoSolution(void){return NoSolution;}
	
	void Kinematics();
	
	void Dump();
	
	void PrintResults();
	
	
	
	
};

#endif
