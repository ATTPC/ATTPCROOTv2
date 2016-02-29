#ifndef TRelativisticKinematics_h
#define TRelativisticKinematics_h 1

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

class TRelativisticKinematics{
	
	private:
	
	double m1; //mass0 of the incident particle
	double m2; //mass0 of the target
	double m3; //mass0 of the scattered particle
	double m4; //mass0 of the recoil
	double ex1; //excitation energy of the incident particle
	double ex2; //excitation energy of the target
	double ex3; //excitation energy of the scattered particle
	double ex4; //excitation energy of the recoil
	
	double tb; // incident energy (total Lab energy in MeV)
	
	double thetacmsInput; // Theta CM angle of the scattered particle, in degrees
	
	double* ANGAs; // lab angle and energy of scattered particle
	double* ANGAr; // lab angle and energy of recoiled particle
	bool    NoSolution;
	
	
	
	public:
	
	TRelativisticKinematics();
	~TRelativisticKinematics();
	
	void SetMassOfProjectile(double value){m1 = value;}
	void SetMassOfTarget(double value){m2 = value;}
	void SetMassOfScattered(double value){m3 = value;}
	void SetMassOfRecoiled(double value){m4 = value;}
	void SetExEnergyOfProjectile(double value){ex1 = value;}
	void SetExEnergyOfTarget(double value){ex2 = value;}
	void SetExEnergyOfScattered(double value){ex3 = value;}
	void SetExEnergyOfRecoiled(double value){ex4 = value;}
	
	void SetLabEnergy(double value){tb = value;}
	
	void SetThetaCMAngle(double value){thetacmsInput = value;}
	
	void SetANGAs(int place, double value){ANGAs[place] = value;}
	void SetANGAr(int place, double value){ANGAr[place] = value;}
	
	void SetNoSolution(bool value){NoSolution=value;}
	
	double GetMassOfProjectile(void){return m1;}
	double GetMassOfTarget(void){return m2;}
	double GetMassOfScattered(void){return m3;}
	double GetMassOfRecoiled(void){return m4;}
	double GetExEnergyOfProjectile(void){return ex1;}
	double GetExEnergyOfTarget(void){return ex2;}
	double GetExEnergyOfScattered(void){return ex3;}
	double GetExEnergyOfRecoiled(void){return ex4;}
	
	double GetLabEnergy(void){return tb;}
	
	double GetThetaCMAngle(void){return thetacmsInput;}
	
	double GetANGAs(int place){return ANGAs[place];}
	double GetANGAr(int place){return ANGAr[place];}
	
	bool GetNoSolution(void){return NoSolution;}
	
	void Kinematics();
	
	void Dump();
	
	void PrintResults();
	
	
	
	
};

#endif
